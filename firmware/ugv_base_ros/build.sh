#!/usr/bin/env bash
#
# Compile ugv_base_ros (patched or stock) for the UGV Rover's ROS Driver
# board -- the original ESP32, "ESP32 Dev Module". Compile only; flashing is
# a hardware-day step (README.md). PLAN-ros-alignment.md 3.28, criterion 1.
#
#   bash firmware/ugv_base_ros/build.sh <checkout of ugv_base_ros>
#
# Needs arduino-cli with the esp32:esp32 core installed. Installs the
# Arduino libraries the firmware includes, then builds into <checkout>/build.
set -euo pipefail

SRC="$(cd "${1:?usage: build.sh <ugv_base_ros checkout>}" && pwd)"
CLI="${ARDUINO_CLI:-$(command -v arduino-cli || echo "$HOME/.local/bin/arduino-cli")}"
FQBN="esp32:esp32:esp32"

# Versions are PINNED: 2e7df97 does not compile against today's latest.
# INA219_WE 1.4 renamed the gain/range/ADC constants battery_ctrl.h uses
# (PG_320, BRNG_16, BIT_MODE_9), and esp32 core 3.3 changed ESP-NOW's send
# callback signature (esp_now_ctrl.h). 1.3.8 and 3.2.1 are the last before
# each change; the rest are what installed on 2026-10-01 (3.28).
CORE="esp32:esp32@3.2.1"
#
# The firmware's README lists Adafruit's ICM libraries, but ROS_Driver.ino
# includes "ICM_20948.h" and calls its DMP API -- SparkFun's library, with
# its DMP support switched on by ICM_20948_USE_DMP (passed as a flag below,
# rather than by editing the installed library).
LIBS=(
  "ArduinoJson@7.4.3"
  "Adafruit SSD1306@2.5.17"
  "Adafruit GFX Library@1.12.6"
  "Adafruit BusIO@1.17.4"
  "Adafruit Unified Sensor@1.1.15"
  "INA219_WE@1.3.8"
  "ESP32Encoder@5.0.0"
  "PID_v2@2.0.1"
  "SimpleKalmanFilter@0.2.0"
  "SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library@1.3.2"
)
"$CLI" core install "$CORE" >/dev/null
for lib in "${LIBS[@]}"; do
  "$CLI" lib install "$lib" >/dev/null
done

# SCServo ships inside the firmware repo, not the library manager.
"$CLI" compile --fqbn "$FQBN" \
  --library "$SRC/SCServo" \
  --build-property "compiler.cpp.extra_flags=-DICM_20948_USE_DMP" \
  --build-property "compiler.c.extra_flags=-DICM_20948_USE_DMP" \
  --output-dir "$SRC/build" \
  "$SRC/ROS_Driver"
