# MLX90640 Thermal Array Integration Design

## Context

The Fiesta pit-radio tire-temperature node currently uses an MLX90614 single-point IR sensor. We are replacing it with an MLX90640 32×24 thermal array to capture a temperature profile across the tire tread.

## Goal

Bring up the MLX90640 on the existing ESP32 hardware, verify wiring and basic sensor operation, and lay the groundwork for a later 5-value tire profile feature.

## Non-goals (this milestone)

- Compute or publish outer/center/inner/average/max values.
- Publish the thermal matrix over MQTT.
- Retain the MLX90614 sensor or driver.

## Architecture

```
app_main()
  └── tire_temp_start()
        └── tire_temp_task()
              ├── I2C bus init (I2C_NUM_0, GPIO 21/22, 400 kHz)
              ├── mlx90640_init()
              └── loop every 1 s
                    ├── mlx90640_read_frame() → 32×24 °C matrix
                    └── log matrix to UART (24 rows, 1 decimal place)
```

### Components

| File | Responsibility |
|------|----------------|
| `main/mlx90640.c`, `main/mlx90640.h` | New driver: I2C communication, EEPROM calibration extraction, frame read, temperature calculation. |
| `main/tire_temp.c`, `main/tire_temp.h` | FreeRTOS task that owns the I2C bus and logs the MLX90640 matrix. |
| `main/main.c` | Unchanged except dropping the MLX90614 include/start. |
| `main/CMakeLists.txt` | Updated source list: remove `mlx90614.c`, add `mlx90640.c`. |

### Removed components

- `main/mlx90614.c`
- `main/mlx90614.h`
- MLX90614 MQTT topic publishing (`fiesta/tire_temp` is reserved for the future 5-value profile).

## Driver API

```c
#define MLX90640_ROWS 24
#define MLX90640_COLS 32

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t addr;
} mlx90640_t;

esp_err_t mlx90640_init(i2c_master_bus_handle_t bus, uint8_t addr, mlx90640_t *out);
esp_err_t mlx90640_read_frame(mlx90640_t *s, float out_temps[MLX90640_ROWS * MLX90640_COLS]);
```

### Implementation notes

- `mlx90640_init()` reads the 832-byte EEPROM, extracts calibration parameters, and stores them in the `mlx90640_t` handle. This is done once; `mlx90640_read_frame()` uses the cached parameters.
- `mlx90640_init()` also configures the sensor refresh rate to 1 Hz.
- Each frame read returns 832 bytes of raw pixel/thermal data.
- The Melexis `CalculateTo` algorithm converts raw data to °C using the extracted parameters.
- All large buffers (EEPROM, frame, parameters) are allocated statically or on the heap inside the driver, not on the caller's stack.
- Emissivity is fixed at `0.95` for rubber tire tread, matching the existing MLX90614 intent.

## I2C configuration

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Port | `I2C_NUM_0` | Same as previous sensor. |
| SDA | GPIO 21 | Existing wiring. |
| SCL | GPIO 22 | Existing wiring. |
| Address | `0x33` | MLX90640 factory default. |
| Speed | `400 kHz` | MLX90640 supports up to 1 MHz; 400 kHz is a safe starting point without the MLX90614 100 kHz constraint. |

## Logging format

Each frame produces 25 log lines:

```
mlx90640: frame 123
mlx90640: row 00: 23.4 23.5 23.4 23.6 ...
mlx90640: row 01: 23.5 23.5 23.6 23.7 ...
...
mlx90640: row 23: 24.1 24.0 24.2 24.1 ...
```

- One decimal place.
- 32 columns per row, space-separated.
- A leading `frame N` marker makes it easy to correlate rows in a noisy log.

## Task configuration

- Task name: `tire_temp`
- Stack size: `8192` bytes (increased from `4096` to accommodate the 32×24 float matrix and calculation buffers).
- Priority: `5` (unchanged).
- Loop period: `1000 ms`.

## Error handling

- If `mlx90640_init()` fails, the task logs the error and retries on the next loop iteration. The firmware does not reboot, so wiring and power can be adjusted live.
- If a frame read fails, the error is logged and the loop continues.
- CRC/PEC-style checks provided by the Melexis algorithm are preserved where applicable.

## Build and verification

### Build changes

1. Remove `mlx90614.c` from `main/CMakeLists.txt`.
2. Add `mlx90640.c` to `main/CMakeLists.txt`.
3. Keep `esp_driver_i2c` in `REQUIRES`.
4. Bump `tire_temp` task stack size in `main/tire_temp.c`.

### Verification steps

1. `idf.py set-target esp32`
2. `idf.py build` passes.
3. Flash and monitor.
4. Confirm the UART output shows a 32×24 matrix with plausible ambient/hot-target values.

## Future work

After wiring verification, the next milestone will:

1. Define configurable ROIs for outer, center, and inner tire zones.
2. Compute average temperature per zone, overall average, and maximum temperature.
3. Publish a JSON payload such as `{"outer":..,"center":..,"inner":..,"avg":..,"max":..}` to `fiesta/tire_temp`.

## Open questions resolved during design

- **Sensor coexistence:** MLX90614 is removed; MLX90640 is the sole sensor.
- **Data output for bring-up:** Full 32×24 matrix logged to UART.
- **Refresh rate:** 1 Hz.
- **Decimal places:** 1.
- **I2C speed:** 400 kHz.
