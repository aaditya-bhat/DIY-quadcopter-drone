#  Quadcopter/Drone

Aaditya Bhat

## Introduction

This project involved designing and building a fully custom quadcopter rather than using an off-the-shelf flight controller. The system was split across three separate microcontrollers: an Arduino Nano running the flight stabilization, an ESP32-CAM handling video and acting as a relay, and an ESP32-S3 running the handheld ground controller. Each board was programmed separately and the three communicate over Wi-Fi and UART to form one control loop.

## System overview

The handheld controller reads the joystick and mode switch and sends this as a UDP packet to the ESP32-CAM. The ESP32-CAM's job on the control side is just to take this packet and forward it over hardware serial to the Nano. Separately, the ESP32-CAM also streams video back to the controller as an MJPEG feed. The Nano is the only board that does anything related to actual flight, everything before it is just moving joystick values and video frames around.

```
Handheld Controller (ESP32-S3)  --UDP-->  ESP32-CAM (relay + video)  --UART, 38400 baud-->  Flight Controller (Nano)
        <--MJPEG stream (port 81)--
```

Splitting the system into three boards instead of one was done so that video encoding and UI rendering, which are both fairly heavy tasks, wouldn't interfere with the timing of the flight control loop. The downside of this is added latency, since a control packet now has to travel over Wi-Fi and then across a UART bridge before it reaches the motors, so a lot of the debugging time in this project went into making sure this link was fast and reliable.

## Hardware used

- Flight controller: Arduino Nano
- Video/relay board: ESP32-CAM (AI Thinker)
- Handheld controller: ESP32-S3 (N16R8)
- Frame: DIY wooden frame
- Motors: A2212 1000kv
- ESCs: 30A
- Propellers: 1045
- IMU: MPU6050
- Rangefinder: VL53L0X
- Barometer: BMP280
- Status LEDs: WS2812 ring, 12 LEDs
- Display: ST7789, 320x240, 8-bit parallel
- UI library: LVGL
- Haptics: PWM-driven vibration motor

The A2212/1045/30A combination was chosen mainly because it's a well-documented, cheap setup that has enough thrust margin to handle the extra weight of the camera, display, and second and third MCU without needing to retune the whole build.

## Flight controller (Arduino Nano)

Roll and pitch are estimated using a complementary filter, combining the gyroscope (fast but drifts over time) with the accelerometer (noisy but doesn't drift), weighted 98% gyro and 2% accelerometer. The dt used in this filter is measured every loop with `micros()` rather than assumed to be fixed, since the loop time isn't perfectly constant.

This feeds into a PID loop tuned at Kp=1.5, Ki=0.04, Kd=0.05. The integral term only accumulates once the throttle is above the arm threshold, and is clamped to ±50, so that it doesn't wind up while the drone is sitting on the ground and cause a lurch on takeoff.

The flight controller moves through four states: DISARMED, FLYING, AUTO_LANDING, and FAULT. Arming is gated with hysteresis, meaning the throttle stick has to drop below one threshold and then be raised past a second, higher threshold before a re-arm is accepted. This was added to fix a race condition where a quick stick movement could re-arm the motors right after a disarm.

If the flight controller stops receiving packets for more than a second, it checks the throttle level. If it was low, it disarms immediately since the drone is probably close to the ground. If it was high, it switches to AUTO_LANDING instead, which ramps the throttle down gradually and uses the laser rangefinder to detect when it's actually reached the ground before disarming.

One of the more important fixes in this project was to the order of operations in `setup()`. The motor pins are now attached and set to a safe 1000us signal as the very first thing that happens, before the IMU is even initialized. This is because the ESCs have their own short arming window right at power-on, and if the signal pins are left floating during that window (for example because the code is still busy calibrating the IMU), some ESCs fail to arm even though the flight controller itself reports armed successfully. After the motors are attached, the IMU is calibrated by averaging 500 gyro and accelerometer samples, and the ground level is calibrated by averaging 50 laser readings, both shown on the LED ring so it's clear when calibration is happening versus done.

Motor commands are mixed in a standard X-quad layout and written out with the Servo library in the 1000-2000us range, capped at 1800us for now during testing so a stuck stick can't command full throttle.

Status LEDs: blue for disarmed, green for flying, blinking yellow for auto-landing, red for fault.

Control packets arrive over hardware serial (D0/D1) at 38400 baud in a simple `<mode,joyX,joyY>` format. This format was kept deliberately basic since it's more robust to bit errors on a serial line than something like JSON would be.

## ESP32-CAM (video + relay)

The camera captures QVGA (320x240) JPEG frames. The streaming task runs pinned to core 1 so it doesn't compete with anything else on the chip, using CAMERA_GRAB_LATEST grab mode with a 3-frame PSRAM buffer, so the buffer always holds the most recent frame instead of the oldest one queued, keeping the feed close to real time. JPEG quality steps down if a frame takes too long to send (over 60ms) and steps back up once sends are consistently fast again. This was added after the feed would noticeably lag on a busier Wi-Fi channel.

On the control side, the ESP32-CAM just listens for UDP packets on port 8888 and forwards them, wrapped in `<...>` framing, to the Nano over Serial1 (GPIO13 RX, GPIO1 TX) at 38400 baud. It doesn't check or interpret the packet at all.

Standard Serial is deliberately not used for debug output on this board. GPIO1 is shared between the USB console and the hardware serial line to the Nano, so if `Serial.begin(115200)` were used to print debug text, that text would also go down the wire to the Nano and get parsed as a control packet, which caused arming/disarming issues before this was figured out. Because of this, all relay traffic goes over Serial1 only.

## Handheld controller (ESP32-S3)

This board is dual-core, and the input/control loop is pinned to Core 0 while the LVGL display rendering runs on Core 1. This was done so the UI could be heavier without adding lag to the control packets being sent out.

The display is a 320x240 ST7789 over 8-bit parallel, using TFT_eSPI as the driver and TJpg_Decoder to decode incoming camera frames, with LVGL handling the UI. There are four screens: boot/calibration, offline, connecting, and the main dashboard. The dashboard shows a live camera view that's scaled, rotated, and centered from the incoming JPEG stream, along with an animated drone attitude indicator for both top-down and side views.

Input comes from an analog joystick and a mode switch. Navigation between screens is done through hold gestures with separate short-hold and long-hold durations, since there wasn't room on the enclosure for more physical buttons.

A vibration motor gives haptic feedback for different events: a short bump, success, error, and a distinct pattern while a hold gesture is in progress.

The controller connects to the drone's Wi-Fi AP, sends control packets over UDP to the ESP32-CAM on port 8888, and pulls the MJPEG stream from the ESP32-CAM's HTTP server on port 81.

## Debugging notes

Motors not spinning after switching to PDB power: after moving to a unified power distribution board, the flight controller would arm successfully (LED went green, state reported FLYING) but the motors wouldn't spin. This was because the ESCs have a short arming window at power-on, and the Arduino hadn't attached the motor pins or sent a valid signal in time, so some ESCs silently failed to arm. Moving the motor attach to the very top of `setup()`, before any sensor init, fixed this.

Unreliable radio link: before switching to hardware serial, the link between the Nano and the ESP32-CAM would drop or corrupt packets intermittently. This came down to two things: bus contention between the USB connection used for uploading and the level converter used for the serial link, and a missing common ground between the two boards, which was easy to miss since both were powered off the same battery but not explicitly tied together on ground. Once hardware serial was used and the ground was confirmed with a multimeter, the link became reliable.

IMU axis inversion: an early version of the complementary filter had the pitch axis inverted relative to what the motor mixing expected, so the drone corrected in the wrong direction under a pitch disturbance. This was caught by comparing the sign of the accelerometer pitch angle against the expected direction on the bench, and has been fixed in both the calibration and filter code.

## Future features

There are a few things in this build that are present in hardware but aren't actually being used by the firmware yet, along with some ideas for where the project could go next.

The BMP280 barometer is a good example of this. It's wired up and readable, but the flight controller currently only uses the VL53L0X laser rangefinder for altitude during auto-landing, since the laser is more accurate at the short ranges involved there. The barometer would be more useful for altitude hold at higher altitudes than the laser is reliable for, since the VL53L0X's range is limited to a few metres before its readings become unreliable. Adding this would mean fusing the barometer reading into the existing filter setup, likely in a similar complementary-filter style to how roll and pitch are already handled, so that the laser is trusted at low altitude and the barometer takes over as the drone climbs higher.

Altitude hold itself is a natural next step once the barometer is in use, where the throttle stick would control climb/descent rate rather than raw throttle directly, and the flight controller would hold a target altitude on its own the rest of the time. This would need a second PID loop running on top of the existing roll/pitch one, using the fused altitude estimate as its input.

A battery voltage monitor would also be worth adding. Right now there's no way for the flight controller to know how much battery is left mid-flight, which means the only warning is the motors starting to sag as the voltage drops. A simple voltage divider into a spare analog pin would let the firmware trigger a low-battery LED warning, and eventually an automatic auto-land if the voltage drops below a safe threshold, similar to how the current failsafe already triggers AUTO_LANDING on loss of signal.

On the handheld controller side, telemetry is currently one-directional, the controller sends joystick input out but doesn't receive anything back from the drone. Adding a return channel so that battery voltage, current flight state, and altitude could be displayed on the dashboard screen would make the existing UI a lot more useful, since right now it's mostly just showing the camera feed and the attitude indicator without any real flight data.

A GPS module is another possibility, mainly for a return-to-home feature in case the radio link is lost for longer than the current auto-land failsafe accounts for, or for position hold outdoors. This would be a bigger addition than the others since it would need its own filtering and a way to convert GPS coordinates into a heading and distance the flight controller could act on.

None of these are implemented yet, they're just documented here so the reasoning behind leaving the barometer unused for now, and the general direction the project could take, is clear to anyone reading this.

## Repository structure

```
/flight-controller/      Arduino Nano firmware (flight_controller.ino)
/esp32-cam-relay/        ESP32-CAM firmware (esp32cam_sender_final.ino)
/handheld-controller/    ESP32-S3 firmware (controller_dashboard_final.ino)
/docs/                   Wiring diagrams, this spec sheet
README.md                Project overview and build instructions
```

Both `esp32cam_sender_final.ino` and `controller_dashboard_final.ino` currently have the Wi-Fi SSID and password hardcoded in plaintext. These have been replaced by placeholders which should be changed.
