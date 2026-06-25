# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

fiesta-tire-temp is an ESP-IDF firmware for an ESP32 that reads tire temperature from an **MLX90640 32×24 thermal array** over I²C. It is part of the larger **Fiesta pit radio system** — a multi-device motorsport telemetry and communication platform. The ESP32 connects to the `fiesta-network` WiFi AP hosted by a Raspberry Pi (carpi), then publishes its online/offline status via MQTT to `broker:1883`.

## Build & Flash

This is an ESP-IDF v5.5+ project. ESP-IDF must be installed and sourced before building.

```bash
# Source ESP-IDF environment (path may vary)
. ~/.espressif/v5.5.3/esp-idf/export.sh

# Set target and build
idf.py set-target esp32
idf.py build

# Flash (default port: /dev/ttyUSB0)
idf.py flash

# Monitor serial output
idf.py monitor

# Build + flash + monitor in one step
idf.py flash monitor
```

There are no unit tests in this project. Verification is `idf.py build` plus hardware-in-the-loop.

## Architecture

The firmware modules are initialized sequentially in `app_main()`:

- **main.c** — Entry point. Initializes NVS and networking, then starts the tire temperature task.
- **wlan.c/h** — WiFi STA connection to `fiesta-network`. Retries up to 3 times, then reboots on failure. Blocks until connected (event group).
- **mqttcomm.c/h** — MQTT client connecting to `mqtt://broker:1883` with client ID `tire-temp`. Publishes online/offline status with LWT (last will and testament) to `fiesta/device/tire-temp/status`. Exposes `mqttcomm_publish()` for general-purpose publishing.
- **tire_temp.c/h** — FreeRTOS task that owns the I²C bus, initializes the MLX90640, reads frames, and logs the 32×24 temperature matrix to UART.
- **mlx90640.c/h** — Driver for the MLX90640 thermal array using the ESP-IDF `i2c_master` API. Includes EEPROM parameter extraction, frame reading, and the Melexis temperature-calculation algorithm.

## Key Configuration

- I²C bus: GPIO 21 (SDA), GPIO 22 (SCL), 400 kHz, sensor address 0x33
- MLX90640 refresh rate: 1 Hz
- `tire_temp` task stack: 8192 bytes
- WiFi SSID/password are `#define`s in `wlan.c` (`WLAN_SSID`, `WLAN_PWD`)
- MQTT broker URI, client ID, and topics are `#define`s in `mqttcomm.c`
- Custom partition table in `partitions.csv` (1MB app, 4KB phy_init, 16KB NVS)

## Current Milestone

The MLX90614 single-point sensor has been replaced with the MLX90640 32×24 array. For this bring-up milestone the firmware logs the full temperature matrix to UART only. The future `fiesta/tire_temp` JSON topic will carry a 5-value tire profile (outer / center / inner / average / max).

## Parent Project Context

See `../PROTOCOL.md` for the full MQTT topic hierarchy and device IDs used across the Fiesta system. The tire temp device ID is `tire-temp`.
