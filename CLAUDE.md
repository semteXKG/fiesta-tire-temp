# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

fiesta-tire-temp is an ESP-IDF firmware for an ESP32 that drives a WS2812 LED strip (9 LEDs on GPIO 12). It is part of the larger **Fiesta pit radio system** — a multi-device motorsport telemetry and communication platform. The ESP32 connects to the `fiesta-network` WiFi AP hosted by a Raspberry Pi (carpi), then publishes its online/offline status via MQTT to `broker:1883`.

## Build & Flash

This is an ESP-IDF v5.5+ project. ESP-IDF must be installed and sourced before building.

```bash
# Source ESP-IDF environment (path may vary)
. ~/.espressif/v5.5.3/esp-idf/export.sh

# Build
idf.py build

# Flash (default port: /dev/ttyUSB0)
idf.py flash

# Monitor serial output
idf.py monitor

# Build + flash + monitor in one step
idf.py flash monitor
```

There are no unit tests in this project.

## Architecture

The firmware has three modules, all initialized sequentially in `app_main()`:

- **main.c** — Entry point. Initializes NVS, networking, LED strip (via `espressif/led_strip` component over RMT), and spawns a FreeRTOS task running a rainbow demo animation.
- **wlan.c/h** — WiFi STA connection to `fiesta-network`. Retries up to 3 times, then reboots on failure. Blocks until connected (event group).
- **mqttcomm.c/h** — MQTT client connecting to `mqtt://broker:1883` with client ID `tire-temp`. Publishes online/offline status with LWT (last will and testament) to `fiesta/device/tire-temp/status`. Exposes `mqttcomm_publish()` for general-purpose publishing.

## Key Configuration

- LED count and GPIO pin are `#define`s in `main.c` (`NUM_LEDS`, `LED_GPIO`)
- WiFi SSID/password are `#define`s in `wlan.c` (`WLAN_SSID`, `WLAN_PWD`)
- MQTT broker URI and topics are `#define`s in `mqttcomm.c`
- Custom partition table in `partitions.csv` (1MB app, 4KB phy_init, 16KB NVS)
- Uses `espressif/led_strip ^2.5.0` managed component

## Parent Project Context

See `../PROTOCOL.md` for the full MQTT topic hierarchy and device IDs used across the Fiesta system. The tire temp device ID is `tire-temp`.
