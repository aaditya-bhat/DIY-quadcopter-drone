# Quadcopter/drone

A fully custom-built quadcopter, built without using an off-the-shelf flight controller. The system is split across three microcontrollers: an Arduino Nano running the flight stabilization, an ESP32-CAM handling video and acting as a relay, and an ESP32-S3 running a handheld ground controller with a screen for FPV.

For the full breakdown of the design decisions, firmware architecture, and debugging history, see 

## Overview

```
Handheld Controller (ESP32-S3)  --UDP-->  ESP32-CAM (relay + video)  --UART, 38400 baud-->  Flight Controller (Nano)
        <--MJPEG stream (port 81)--
```

The handheld controller reads the joystick and mode switch and sends this over UDP to the ESP32-CAM. The ESP32-CAM forwards this over hardware serial to the Nano, and separately streams video back to the controller as an MJPEG feed. The Nano is the only board making actual flight decisions.

## Hardware

- Flight controller: Arduino Nano
- Video/relay board: ESP32-CAM (AI Thinker)
- Handheld controller: ESP32-S3 (N8R8)
- Frame: DIY wooden frame
- Motors: A2212 1000kv
- ESCs: 30A
- Propellers: 1045
- IMU: MPU6050
- Rangefinder: VL53L0X
- Barometer: BMP280
- Status LEDs: WS2812 ring, 12 LEDs
- Display: ST7789, 320x240, 8-bit parallel
- Haptics: PWM-driven vibration motor

## Repository structure

```
/flight-controller/      Arduino Nano firmware (flight_controller.ino)
/esp32-cam-relay/        ESP32-CAM firmware (esp32cam_sender_final.ino)
/handheld-controller/    ESP32-S3 firmware (controller_dashboard_final.ino)
/docs/                   Wiring diagrams, full spec sheet (SPEC.md)
```

## Flashing the firmware

Each board is programmed separately from the Arduino IDE.

1. `/flight-controller/flight_controller.ino` — select Arduino Nano as the board, flash over USB.
2. `/esp32-cam-relay/esp32cam_sender_final.ino` — select the AI Thinker ESP32-CAM board, flash over a USB-to-serial adapter (GPIO0 tied low during upload).
3. `/handheld-controller/controller_dashboard_final.ino` — select the appropriate ESP32-S3 board, flash over USB.

Before flashing the ESP32-CAM and handheld controller firmware, the Wi-Fi SSID and password need to be set. These are currently placeholders in this repo — see the note below.

## Status

This project is still being actively flight-tested. Known issues and recent fixes are documented in [`docs/SPEC.md`](docs/SPEC.md#debugging-notes).
