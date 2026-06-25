# AGENTS.md

ESP-IDF v5.5 firmware for an ESP32 in the Fiesta pit-radio system. Reads tire temperature from an MLX90640 32×24 IR thermal array over I²C and publishes readings plus device status over MQTT.

## Build

ESP-IDF must be installed and sourced first. This repo targets `esp32` (not `esp32s3`).

```bash
. ~/.espressif/v5.5.3/esp-idf/export.sh
idf.py set-target esp32   # required on fresh clones; target must be esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

There are no unit tests. Verification is `idf.py build` plus hardware-in-the-loop.

## Architecture

`app_main()` in `main/main.c` initializes NVS and netif, then starts modules in order:

- `wlan.c/h` — STA connection to `fiesta-network`. Blocks until connected; reboots after 3 failed attempts.
- `mqttcomm.c/h` — MQTT client to `mqtt://broker:1883`, client ID `tire-temp`. Publishes LWT status to `fiesta/device/tire-temp/status`; exposes `mqttcomm_publish()`.
- `tire_temp.c/h` — FreeRTOS task that reads an MLX90640 on I²C (GPIO 21 SDA, GPIO 22 SCL) every 1 s and logs the full 32×24 temperature matrix to UART. In this bring-up milestone it does not publish the matrix over MQTT.
- `mlx90640.c/h` — Driver for the MLX90640 32×24 thermal array using the ESP-IDF 5.2+ `i2c_master` API. Includes EEPROM parameter extraction, frame reading, and the Melexis temperature-calculation algorithm.

## Key configuration

- WiFi SSID/password: hardcoded in `wlan.c`.
- MQTT broker URI, client ID, topics: hardcoded in `mqttcomm.c`.
- I²C pins (GPIO 21 SDA, GPIO 22 SCL), sensor address (0x33), poll period (1 s), and refresh rate (1 Hz): hardcoded in `tire_temp.c`.
- Partition table: `partitions.csv` (1 MB app, 16 KB NVS).
- Component deps: `main/idf_component.yml` requires IDF `>=5.5.2`; `main/CMakeLists.txt` requires `esp_driver_i2c`.

## Gotchas

- `CLAUDE.md` describes the MLX90640 32×24 thermal array; the old WS2812 LED content has been replaced.
- `partitions.csv` is automatically enabled on fresh clones via `sdkconfig.defaults` (`CONFIG_PARTITION_TABLE_CUSTOM=y`). If you have an existing `sdkconfig` from before this file was added, run `idf.py reconfigure` or delete `sdkconfig` to pick it up.
- There is no committed `sdkconfig`; `sdkconfig.old` is gitignored and may not reflect the active config.
- The new I²C master driver (`driver/i2c_master.h`) is used; do not revert to the legacy `driver/i2c.h` API or remove `esp_driver_i2c` from `main/CMakeLists.txt`.
- The broker hostname `broker` relies on the pit network's DNS/local resolution (usually the carpi Pi).
