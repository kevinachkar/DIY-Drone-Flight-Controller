# DIY Drone Flight Controller

A custom ESP32-C3 quadcopter project with a 3D-printed frame designed in Onshape, a custom PCB designed in KiCad, and flight-control firmware written in Arduino C++.

**Reported flight results:** approximately **270 g** including the battery and **15–18 minutes** of flight with a **2S 2500 mAh LiPo**.

## Current progress

| Feature | Status |
| --- | --- |
| Manual flight with attitude stabilization | Flying well; full pilot control |
| Altitude hold | Implemented; requires hover-throttle calibration and tuning |
| GPS position hold | Experimental; further tuning needed |
| Return to home (RTH) | Experimental; further tuning needed |

With little or no wind and stable GPS reception, position hold has roughly kept the drone within a 3–4 m area in the builder's tests. This is an observation, not a guaranteed accuracy specification.

## Control link

RadioMaster Pocket (USB joystick) → laptop Python script → USB serial → Arduino Nano → nRF24L01 radio → ESP32-C3 drone.

The laptop and Nano are part of the current control link; the Pocket does not transmit directly to the drone's nRF24L01 receiver.

## Repository contents

| Folder | Contents |
| --- | --- |
| [firmware](firmware/) | Main ESP32-C3 flight-controller sketch |
| [libraries](libraries/) | Four custom Arduino libraries |
| [transmitter](transmitter/) | Arduino Nano serial-to-nRF24 transmitter |
| [ground-control](ground-control/) | Python USB-controller bridge and Python dependencies |
| [docs](docs/) | Installation, controls, and GitHub upload instructions |
| [media](media/) | Instructions for adding photos and flight videos |

## Custom libraries

- **MPU6050_Mahony_DynamicNotch** — IMU attitude estimation and optional dynamic notch filtering.
- **NonBlockingBaro** — non-blocking barometer reading and relative altitude.
- **TiltCompensatedMag** — magnetometer heading with tilt compensation.
- **KalmanVertVel** — vertical velocity estimation from IMU and barometer measurements.

## Getting started

See [setup and controls](docs/SETUP.md). See [upload instructions](docs/GITHUB_UPLOAD.md) to add this folder and your media to GitHub.

## Flight media

Add your photo to `media/photos/IMG_8073.jpg`, then insert this line here:

```markdown
![Custom DIY drone](media/photos/IMG_8073.jpg)
```

Add flight-video links here after uploading the videos. Label each clip according to what it actually shows.

## Development notes

Position hold and RTH are still being developed. PID gains, sensor alignment, calibration, wiring, and switch mappings are specific to this build. The supplied source has been organized without intentional flight-control logic changes. This package has not been compiled or flight-tested.

The Onshape frame files and KiCad PCB design files are not included in this source package.

