/*
 * MLX90640 thermal array driver for ESP-IDF (new i2c_master API)
 * 32x24 IR array, I2C interface.
 *
 * Copyright 2021 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The temperature-calculation algorithm and EEPROM parameter extraction
 * are adapted from the Melexis MLX90640 reference driver.
 */
#ifndef MLX90640_H
#define MLX90640_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MLX90640_ROWS 24
#define MLX90640_COLS 32
#define MLX90640_PIXELS (MLX90640_ROWS * MLX90640_COLS)

#define MLX90640_DEFAULT_ADDR 0x33

/* Calibration parameters extracted from EEPROM (Melexis layout). */
typedef struct {
    int16_t kVdd;
    int16_t vdd25;
    float KvPTAT;
    float KtPTAT;
    uint16_t vPTAT25;
    float alphaPTAT;
    int16_t gainEE;
    float tgc;
    float cpKv;
    float cpKta;
    uint8_t resolutionEE;
    uint8_t calibrationModeEE;
    float KsTa;
    float ksTo[5];
    int16_t ct[5];
    uint16_t alpha[MLX90640_PIXELS];
    uint8_t alphaScale;
    int16_t offset[MLX90640_PIXELS];
    int8_t kta[MLX90640_PIXELS];
    uint8_t ktaScale;
    int8_t kv[MLX90640_PIXELS];
    uint8_t kvScale;
    float cpAlpha[2];
    int16_t cpOffset[2];
    float ilChessC[3];
    uint16_t brokenPixels[5];
    uint16_t outlierPixels[5];
} mlx90640_params_t;

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t addr;
    mlx90640_params_t params;
} mlx90640_t;

/* Attach the sensor at `addr` to an existing I2C master bus and read EEPROM params.
   This driver is NOT reentrant: only one task may call it at a time. */
esp_err_t mlx90640_init(i2c_master_bus_handle_t bus, uint8_t addr, mlx90640_t *out);

/* Read one full frame and convert to Celsius in `out_temps[768]`.
   emissivity: surface emissivity (0.0..1.0], e.g. 0.95 for rubber.
   reflected_temp_c: reflected ambient temperature in Celsius.
   This driver is NOT reentrant: only one task may call it at a time. */
esp_err_t mlx90640_read_frame(mlx90640_t *s, float out_temps[MLX90640_PIXELS],
                               float emissivity, float reflected_temp_c);

#ifdef __cplusplus
}
#endif

#endif /* MLX90640_H */
