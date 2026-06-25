# MLX90640 Thermal Array Integration Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the MLX90614 sensor with an MLX90640 32×24 thermal array, verify wiring by logging the full temperature matrix to UART.

**Architecture:** A new `mlx90640.c/h` driver uses the ESP-IDF `i2c_master` API to read EEPROM calibration data and frame data, then runs the Melexis temperature-calculation algorithm. `tire_temp.c` owns the I2C bus and a FreeRTOS task that reads one frame per second and logs the 32×24 matrix.

**Tech Stack:** ESP-IDF v5.5.x, ESP32 target, new `i2c_master` driver (`driver/i2c_master.h`), FreeRTOS, ESP-IDF logging.

## Global Constraints

- Target: `esp32` (not `esp32s3`).
- IDF version: `>=5.5.2`.
- I2C API: new `driver/i2c_master.h`; do not use legacy `driver/i2c.h`.
- I2C pins: SDA GPIO 21, SCL GPIO 22.
- MLX90640 I2C address: `0x33`.
- I2C speed: `400 kHz`.
- Refresh rate: `1 Hz`.
- Matrix log: 24 rows, 32 columns per row, 1 decimal place.
- `tire_temp` task stack: `8192` bytes.
- No MQTT publishing of the matrix in this milestone.
- Remove `main/mlx90614.c` and `main/mlx90614.h` completely.

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `main/mlx90614.c` | Delete | Old single-point driver. |
| `main/mlx90614.h` | Delete | Old driver header. |
| `main/mlx90640.h` | Create | Public driver types and API. |
| `main/mlx90640.c` | Create | I2C SMBus wrapper + Melexis algorithm. |
| `main/tire_temp.c` | Modify | Use MLX90640 instead of MLX90614. |
| `main/CMakeLists.txt` | Modify | Replace `mlx90614.c` with `mlx90640.c`. |
| `main/main.c` | No change | Already calls `tire_temp_start()` only. |

---

### Task 1: Remove MLX90614 driver and update build list

**Files:**
- Delete: `main/mlx90614.c`
- Delete: `main/mlx90614.h`
- Modify: `main/CMakeLists.txt`

**Interfaces:**
- Produces: `main/CMakeLists.txt` no longer references `mlx90614.c`.

- [ ] **Step 1: Delete old driver files**

```bash
rm main/mlx90614.c main/mlx90614.h
```

- [ ] **Step 2: Update `main/CMakeLists.txt`**

Replace the existing line with:

```cmake
idf_component_register(SRCS "main.c" "wlan.c" "mqttcomm.c" "mlx90640.c" "tire_temp.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver mqtt nvs_flash esp_netif esp_event esp_wifi esp_driver_i2c)
```

- [ ] **Step 3: Commit**

```bash
git add main/CMakeLists.txt
git rm main/mlx90614.c main/mlx90614.h
git commit -m "build: remove mlx90614, add mlx90640 to source list"
```

---

### Task 2: Create `main/mlx90640.h`

**Files:**
- Create: `main/mlx90640.h`

**Interfaces:**
- Produces: `mlx90640_t`, `MLX90640_ROWS`, `MLX90640_COLS`, `mlx90640_init()`, `mlx90640_read_frame()`.
- Consumes: nothing.

- [ ] **Step 1: Write header file**

```c
/*
 * MLX90640 thermal array driver for ESP-IDF (new i2c_master API)
 * 32x24 IR array, I2C interface.
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

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t addr;
    /* Calibration parameters extracted from EEPROM */
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
    float ksTo[4];
    int16_t ct[4];
    float alpha[MLX90640_PIXELS];
    int16_t offset[MLX90640_PIXELS];
    float kta[MLX90640_PIXELS];
    float kv[MLX90640_PIXELS];
    float cpAlpha[2];
    int16_t cpOffset[2];
    float ilChessC[3];
    uint16_t brokenPixels[5];
    uint16_t outlierPixels[5];
} mlx90640_t;

/* Attach the sensor at `addr` to an existing I2C master bus and read EEPROM params. */
esp_err_t mlx90640_init(i2c_master_bus_handle_t bus, uint8_t addr, mlx90640_t *out);

/* Read one full frame and convert to Celsius in `out_temps[768]`. */
esp_err_t mlx90640_read_frame(mlx90640_t *s, float out_temps[MLX90640_PIXELS]);

#ifdef __cplusplus
}
#endif

#endif /* MLX90640_H */
```

- [ ] **Step 2: Commit**

```bash
git add main/mlx90640.h
git commit -m "feat: add mlx90640 public driver header"
```

---

### Task 3: Create `main/mlx90640.c` — I2C wrapper and EEPROM parameter extraction

**Files:**
- Create: `main/mlx90640.c`

**Interfaces:**
- Consumes: `i2c_master` API, `mlx90640.h`.
- Produces: `mlx90640_init()` that fills the `mlx90640_t` calibration fields.

- [ ] **Step 1: Write the first half of the driver**

```c
#include "mlx90640.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "mlx90640";

#define I2C_TIMEOUT_MS 100

/* Melexis register addresses */
#define MLX90640_REG_EEPROM     0x2400
#define MLX90640_REG_FRAME0     0x0400
#define MLX90640_REG_FRAME1     0x0700
#define MLX90640_REG_CTRL       0x800D
#define MLX90640_REG_STATUS     0x8000

/* Forward declarations */
static esp_err_t mlx90640_read_register(mlx90640_t *s, uint16_t reg, uint16_t *value);
static esp_err_t mlx90640_write_register(mlx90640_t *s, uint16_t reg, uint16_t value);
static void extract_parameters(uint16_t *eeData, mlx90640_t *p);
static int check_eeprom_valid(uint16_t *eeData);
static void mlx90640_calculate_to(mlx90640_t *p, uint16_t *frameData, float emissivity, float *result);

esp_err_t mlx90640_init(i2c_master_bus_handle_t bus, uint8_t addr, mlx90640_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &out->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to add device: %s", esp_err_to_name(err));
        return err;
    }
    out->addr = addr;

    /* Read full EEPROM (832 bytes = 416 words) */
    uint16_t eeData[832];
    uint8_t cmd[2] = { (uint8_t)(MLX90640_REG_EEPROM >> 8),
                       (uint8_t)(MLX90640_REG_EEPROM & 0xFF) };
    err = i2c_master_transmit_receive(out->dev, cmd, sizeof(cmd),
                                      (uint8_t *)eeData, sizeof(eeData),
                                      I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "EEPROM read failed: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(out->dev);
        return err;
    }

    /* EEPROM is big-endian; ESP32 is little-endian. Swap each word. */
    for (int i = 0; i < 832; i++) {
        eeData[i] = (eeData[i] << 8) | (eeData[i] >> 8);
    }

    if (!check_eeprom_valid(eeData)) {
        ESP_LOGE(TAG, "EEPROM data invalid");
        i2c_master_bus_rm_device(out->dev);
        return ESP_ERR_INVALID_RESPONSE;
    }

    extract_parameters(eeData, out);

    /* Set refresh rate to 1 Hz (bits 7:9 in control register = 001 = 1Hz) */
    uint16_t ctrl;
    err = mlx90640_read_register(out, MLX90640_REG_CTRL, &ctrl);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(out->dev);
        return err;
    }
    ctrl &= ~0x0380;          /* clear refresh bits [9:7] */
    ctrl |= 0x0080;           /* 001 = 1 Hz */
    err = mlx90640_write_register(out, MLX90640_REG_CTRL, ctrl);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(out->dev);
        return err;
    }

    ESP_LOGI(TAG, "MLX90640 initialized @ 0x%02X", addr);
    return ESP_OK;
}

esp_err_t mlx90640_read_register(mlx90640_t *s, uint16_t reg, uint16_t *value)
{
    uint8_t cmd[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    uint8_t rx[2];
    esp_err_t err = i2c_master_transmit_receive(s->dev, cmd, sizeof(cmd),
                                                rx, sizeof(rx), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    *value = ((uint16_t)rx[0] << 8) | rx[1];
    return ESP_OK;
}

esp_err_t mlx90640_write_register(mlx90640_t *s, uint16_t reg, uint16_t value)
{
    uint8_t tx[4] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
                      (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return i2c_master_transmit(s->dev, tx, sizeof(tx), I2C_TIMEOUT_MS);
}

static int check_eeprom_valid(uint16_t *eeData)
{
    /* Simple sanity: device ID / config word at 0x240A should be non-zero and not all ones */
    if (eeData[0x0A] == 0x0000 || eeData[0x0A] == 0xFFFF) {
        return 0;
    }
    return 1;
}

static void extract_parameters(uint16_t *eeData, mlx90640_t *p)
{
    int resolutionEE;
    int calibrationModeEE;

    p->kVdd = (eeData[51] & 0xFF00) >> 8;
    if (p->kVdd > 127) {
        p->kVdd = p->kVdd - 256;
    }
    p->kVdd = p->kVdd * 32;

    p->vdd25 = eeData[51] & 0x00FF;
    p->vdd25 = ((p->vdd25 - 256) << 5) - 8192;

    p->KvPTAT = (eeData[50] & 0xFC00) >> 10;
    if (p->KvPTAT > 31) {
        p->KvPTAT = p->KvPTAT - 64;
    }
    p->KvPTAT = p->KvPTAT / 4096.0f;

    p->KtPTAT = eeData[50] & 0x03FF;
    if (p->KtPTAT > 511) {
        p->KtPTAT = p->KtPTAT - 1024;
    }
    p->KtPTAT = p->KtPTAT / 8.0f;

    p->vPTAT25 = eeData[49];
    if (p->vPTAT25 > 32767) {
        p->vPTAT25 = p->vPTAT25 - 65536;
    }

    p->alphaPTAT = (eeData[16] & 0xF000) / powf(2.0f, (float)(14));
    if (p->alphaPTAT > 7) {
        p->alphaPTAT = p->alphaPTAT - 16;
    }
    p->alphaPTAT = p->alphaPTAT * powf(2.0f, (float)(eeData[16] & 0x0FFF));

    p->gainEE = eeData[48];
    if (p->gainEE > 32767) {
        p->gainEE = p->gainEE - 65536;
    }

    p->tgc = eeData[60] & 0x00FF;
    if (p->tgc > 127) {
        p->tgc = p->tgc - 256;
    }
    p->tgc = p->tgc / 32.0f;

    p->cpKv = (eeData[52] & 0xFF00) >> 8;
    if (p->cpKv > 127) {
        p->cpKv = p->cpKv - 256;
    }
    p->cpKv = p->cpKv / 256.0f;

    p->cpKta = eeData[52] & 0x00FF;
    if (p->cpKta > 127) {
        p->cpKta = p->cpKta - 256;
    }
    p->cpKta = p->cpKta / 2048.0f;

    resolutionEE = (eeData[56] & 0x3000) >> 12;
    p->resolutionEE = (uint8_t)resolutionEE;

    calibrationModeEE = (eeData[10] & 0x0800) >> 11;
    p->calibrationModeEE = (uint8_t)calibrationModeEE;

    p->KsTa = (eeData[60] & 0xFF00) >> 8;
    if (p->KsTa > 127) {
        p->KsTa = p->KsTa - 256;
    }
    p->KsTa = p->KsTa / 8192.0f;

    p->ksTo[0] = eeData[61] & 0x00FF;
    if (p->ksTo[0] > 127) {
        p->ksTo[0] = p->ksTo[0] - 256;
    }
    p->ksTo[0] = p->ksTo[0] / 1024.0f;

    p->ksTo[1] = (eeData[61] & 0xFF00) >> 8;
    if (p->ksTo[1] > 127) {
        p->ksTo[1] = p->ksTo[1] - 256;
    }
    p->ksTo[1] = p->ksTo[1] / 1024.0f;

    p->ksTo[2] = eeData[62] & 0x00FF;
    if (p->ksTo[2] > 127) {
        p->ksTo[2] = p->ksTo[2] - 256;
    }
    p->ksTo[2] = p->ksTo[2] / 1024.0f;

    p->ksTo[3] = (eeData[62] & 0xFF00) >> 8;
    if (p->ksTo[3] > 127) {
        p->ksTo[3] = p->ksTo[3] - 256;
    }
    p->ksTo[3] = p->ksTo[3] / 1024.0f;

    p->ct[0] = -40;
    p->ct[1] = 0;
    p->ct[2] = (eeData[63] & 0x00F0) >> 4;
    p->ct[3] = (eeData[63] & 0x0F00) >> 8;

    /* Per-pixel alpha, offset, kta, kv */
    float ktaScale = powf(2.0f, (float)(eeData[56] & 0x000F));
    float kvScale = powf(2.0f, (float)((eeData[56] & 0x00F0) >> 4));
    float alphaScale = powf(2.0f, (float)((eeData[32] & 0xF000) >> 12));
    int offsetRef = eeData[63] & 0x003F;
    if (offsetRef > 31) {
        offsetRef = offsetRef - 64;
    }

    for (int i = 0; i < MLX90640_PIXELS; i++) {
        /* alpha */
        p->alpha[i] = (eeData[64 + i] & 0x03F0) >> 4;
        if (p->alpha[i] > 31) {
            p->alpha[i] = p->alpha[i] - 64;
        }
        p->alpha[i] = p->alpha[i] / powf(2.0f, (float)(eeData[64 + i] & 0x000F));
        p->alpha[i] = p->alpha[i] / alphaScale;

        /* offset */
        p->offset[i] = (eeData[64 + i] & 0xFC00) >> 10;
        if (p->offset[i] > 31) {
            p->offset[i] = p->offset[i] - 64;
        }
        p->offset[i] = p->offset[i] + offsetRef;

        /* kta */
        p->kta[i] = (eeData[64 + i] & 0x000E) >> 1;
        if (p->kta[i] > 3) {
            p->kta[i] = p->kta[i] - 8;
        }
        p->kta[i] = p->kta[i] / ktaScale;

        /* kv */
        p->kv[i] = eeData[64 + i] & 0x0001;
        if (p->kv[i] > 0) {
            p->kv[i] = p->kv[i] - 2;
        }
        p->kv[i] = p->kv[i] / kvScale;
    }

    /* Compensation pixels */
    p->cpAlpha[0] = (eeData[56] & 0xFC00) >> 10;
    if (p->cpAlpha[0] > 31) {
        p->cpAlpha[0] = p->cpAlpha[0] - 64;
    }
    p->cpAlpha[0] = p->cpAlpha[0] / powf(2.0f, (float)(eeData[56] & 0x003F));
    p->cpAlpha[0] = p->cpAlpha[0] / alphaScale;

    p->cpAlpha[1] = (eeData[57] & 0xFC00) >> 10;
    if (p->cpAlpha[1] > 31) {
        p->cpAlpha[1] = p->cpAlpha[1] - 64;
    }
    p->cpAlpha[1] = p->cpAlpha[1] / powf(2.0f, (float)(eeData[57] & 0x003F));
    p->cpAlpha[1] = p->cpAlpha[1] / alphaScale;

    p->cpOffset[0] = eeData[56] & 0x03FF;
    if (p->cpOffset[0] > 511) {
        p->cpOffset[0] = p->cpOffset[0] - 1024;
    }

    p->cpOffset[1] = eeData[57] & 0x03FF;
    if (p->cpOffset[1] > 511) {
        p->cpOffset[1] = p->cpOffset[1] - 1024;
    }

    p->cpKta = eeData[58] & 0x00FF;
    if (p->cpKta > 127) {
        p->cpKta = p->cpKta - 256;
    }
    p->cpKta = p->cpKta / 2048.0f;

    p->cpKv = (eeData[58] & 0xFF00) >> 8;
    if (p->cpKv > 127) {
        p->cpKv = p->cpKv - 256;
    }
    p->cpKv = p->cpKv / 256.0f;

    /* Interleaved chess constants */
    p->ilChessC[0] = (eeData[59] & 0x003F);
    if (p->ilChessC[0] > 31) {
        p->ilChessC[0] = p->ilChessC[0] - 64;
    }
    p->ilChessC[0] = p->ilChessC[0] / 16.0f;

    p->ilChessC[1] = (eeData[59] & 0x07C0) >> 6;
    if (p->ilChessC[1] > 15) {
        p->ilChessC[1] = p->ilChessC[1] - 32;
    }
    p->ilChessC[1] = p->ilChessC[1] / 2.0f;

    p->ilChessC[2] = (eeData[59] & 0xF800) >> 11;
    if (p->ilChessC[2] > 15) {
        p->ilChessC[2] = p->ilChessC[2] - 32;
    }
    p->ilChessC[2] = p->ilChessC[2] / 8.0f;

    /* Broken/outlier pixels: not handled in this minimal port */
    memset(p->brokenPixels, 0, sizeof(p->brokenPixels));
    memset(p->outlierPixels, 0, sizeof(p->outlierPixels));
}
```

- [ ] **Step 2: Commit**

```bash
git add main/mlx90640.c
git commit -m "feat: add mlx90640 init and eeprom parameter extraction"
```

---

### Task 4: Add frame read and temperature calculation to `main/mlx90640.c`

**Files:**
- Modify: `main/mlx90640.c`

**Interfaces:**
- Consumes: calibration fields in `mlx90640_t` from Task 3.
- Produces: `mlx90640_read_frame()`.

- [ ] **Step 1: Append the frame read and calculation code**

Add the following at the bottom of `main/mlx90640.c`:

```c
esp_err_t mlx90640_read_frame(mlx90640_t *s, float out_temps[MLX90640_PIXELS])
{
    if (out_temps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t frameData[832];
    uint16_t status;
    uint16_t ctrl;
    esp_err_t err;

    /* Determine which frame buffer is active */
    err = mlx90640_read_register(s, MLX90640_REG_CTRL, &ctrl);
    if (err != ESP_OK) {
        return err;
    }
    int frameSel = (ctrl & 0x0001) ? 1 : 0;
    uint16_t frameAddr = frameSel ? MLX90640_REG_FRAME1 : MLX90640_REG_FRAME0;

    /* Wait for new data (bit 0 in status register toggles) */
    for (int attempt = 0; attempt < 100; attempt++) {
        err = mlx90640_read_register(s, MLX90640_REG_STATUS, &status);
        if (err != ESP_OK) {
            return err;
        }
        if ((status & 0x0001) == frameSel) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Read frame: 832 bytes = 416 words */
    uint8_t cmd[2] = { (uint8_t)(frameAddr >> 8), (uint8_t)(frameAddr & 0xFF) };
    err = i2c_master_transmit_receive(s->dev, cmd, sizeof(cmd),
                                      (uint8_t *)frameData, sizeof(frameData),
                                      I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "frame read failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Byte-swap frame data */
    for (int i = 0; i < 832; i++) {
        frameData[i] = (frameData[i] << 8) | (frameData[i] >> 8);
    }

    /* Clear new-data bit */
    status &= ~0x0001;
    mlx90640_write_register(s, MLX90640_REG_STATUS, status);

    /* Calculate temperatures */
    mlx90640_calculate_to(s, frameData, 0.95f, out_temps);
    return ESP_OK;
}

static float get_vdd(uint16_t *frameData, mlx90640_t *p)
{
    float vdd;
    float resolutionCorrection;

    int resolutionRAM = (frameData[832 - 4] & 0x0C00) >> 10;
    resolutionCorrection = powf(2.0f, (float)(p->resolutionEE - resolutionRAM));

    vdd = (float)(resolutionCorrection * (int16_t)frameData[832 - 3] - p->vdd25) / p->kVdd + 3.3f;

    return vdd;
}

static float get_ta(uint16_t *frameData, mlx90640_t *p)
{
    float ptat;
    float ptatArt;
    float vdd;
    float ta;

    vdd = get_vdd(frameData, p);

    ptat = (float)((int16_t)frameData[832 - 2]);
    ptatArt = (float)((int16_t)frameData[832 - 1]);

    ptatArt = (ptat / (ptat * p->alphaPTAT + ptatArt)) * powf(2.0f, 18.0f);
    ta = ptatArt / (1.0f + p->KvPTAT * (vdd - 3.3f));
    ta = ta - p->vPTAT25;
    ta = ta / p->KtPTAT + 25.0f;

    return ta;
}

static void mlx90640_calculate_to(mlx90640_t *p, uint16_t *frameData, float emissivity, float *result)
{
    float vdd;
    float ta;
    float ta4;
    float tr4;
    float taTr;
    float gain;
    float irDataCP[2];
    float irData;
    float alphaCompensated;
    float sx;
    float to;
    float alphaCorrR[4];
    int mode;
    int8_t ilPattern;
    int8_t chessPattern;
    int8_t pattern;
    int8_t conversionPattern;
    float Sx;
    float To;
    float alphaCorr;
    int row;
    int column;

    vdd = get_vdd(frameData, p);
    ta = get_ta(frameData, p);

    ta4 = powf((ta + 273.15f), 4.0f);
    tr4 = ta4;
    taTr = tr4 - (tr4 - ta4) / emissivity;

    alphaCorrR[0] = 1 / (1 + p->ksTo[0] * 40);
    alphaCorrR[1] = 1;
    alphaCorrR[2] = (1 + p->ksTo[2] * p->ct[2]);
    alphaCorrR[3] = alphaCorrR[2] * (1 + p->ksTo[3] * (p->ct[3] - p->ct[2]));

    /* Gain compensation */
    gain = (float)p->gainEE / (int16_t)frameData[832 - 4];

    /* Mode: 0 = chess, 1 = TV */
    mode = (frameData[832 - 4] & 0x1000) >> 12;

    /* Compensation pixel processing */
    irDataCP[0] = (int16_t)frameData[832 - 2];
    irDataCP[1] = (int16_t)frameData[832 - 1];

    irDataCP[0] = irDataCP[0] - (p->cpOffset[0] + p->cpKta * (ta - 25.0f)) * (1 + p->cpKv * (vdd - 3.3f));
    irDataCP[1] = irDataCP[1] - (p->cpOffset[1] + p->cpKta * (ta - 25.0f)) * (1 + p->cpKv * (vdd - 3.3f));

    irDataCP[0] = irDataCP[0] * gain;
    irDataCP[1] = irDataCP[1] * gain;

    float cpAlpha[2];
    cpAlpha[0] = p->cpAlpha[0];
    cpAlpha[1] = p->cpAlpha[1];

    for (int i = 0; i < MLX90640_PIXELS; i++) {
        ilPattern = (i >> 5) - (i >> 3) * (i >> 4);
        chessPattern = ilPattern ^ (i - (i >> 2) * 4);
        conversionPattern = ((i + 2) >> 2) - ((i + 3) >> 2) + ((i + 1) >> 2) - (i >> 2);

        if (mode == 0) {
            pattern = chessPattern;
        } else {
            pattern = ilPattern;
        }

        if (pattern == p->calibrationModeEE) {
            irData = (int16_t)frameData[i];
        } else {
            irData = (int16_t)frameData[i] - (p->ilChessC[conversionPattern] * (1 + p->ilChessC[2] * (ta - 25.0f)));
        }

        irData = irData * gain;

        irData = irData - p->offset[i] * (1 + p->kta[i] * (ta - 25.0f)) * (1 + p->kv[i] * (vdd - 3.3f));

        irData = irData - p->tgc * irDataCP[pattern];

        alphaCompensated = (p->alpha[i] - p->tgc * cpAlpha[pattern]) * (1 + p->KsTa * (ta - 25.0f));

        sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
        Sx = sx * (irData + alphaCompensated * taTr);
        Sx = sqrtf(Sx) * 25.0f;

        To = powf((irData / (alphaCompensated * (1 - p->ksTo[1] * 273.15f) + Sx) + taTr), 0.25f);

        alphaCorr = alphaCompensated * (1 + p->ksTo[1] * (To - 273.15f));

        if (To < p->ct[1]) {
            alphaCorr = alphaCorr * alphaCorrR[0];
        } else if (To < p->ct[2]) {
            alphaCorr = alphaCorr * alphaCorrR[1];
        } else if (To < p->ct[3]) {
            alphaCorr = alphaCorr * alphaCorrR[2];
        } else {
            alphaCorr = alphaCorr * alphaCorrR[3];
        }

        To = powf((irData / (alphaCorr * (1 - p->ksTo[1] * 273.15f) + Sx) + taTr), 0.25f);
        result[i] = To - 273.15f;
    }
}
```

Note: Forward declarations were already added in Task 3, so the new functions can call each other in any order.

- [ ] **Step 2: Commit**

```bash
git add main/mlx90640.c
git commit -m "feat: add mlx90640 frame read and temperature calculation"
```

---

### Task 5: Update `main/tire_temp.c` for MLX90640 logging

**Files:**
- Modify: `main/tire_temp.c`

**Interfaces:**
- Consumes: `mlx90640.h`, `mlx90640_init()`, `mlx90640_read_frame()`.
- Produces: updated `tire_temp_task()` that logs the matrix.

- [ ] **Step 1: Replace the contents of `main/tire_temp.c`**

```c
#include "tire_temp.h"
#include "mlx90640.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define I2C_SDA_GPIO     21
#define I2C_SCL_GPIO     22
#define POLL_PERIOD_MS   1000
#define SENSOR_ADDR      0x33

static const char *TAG = "tire_temp";

static void tire_temp_task(void *pv)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    mlx90640_t sensor;
    float matrix[MLX90640_PIXELS];
    int frame_count = 0;

    while (1) {
        esp_err_t err = mlx90640_init(bus, SENSOR_ADDR, &sensor);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "MLX90640 init failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
            continue;
        }

        ESP_LOGI(TAG, "MLX90640 attached @ 0x%02X", SENSOR_ADDR);

        /* Read loop. If read fails, re-init on next outer iteration. */
        while (1) {
            err = mlx90640_read_frame(&sensor, matrix);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "frame read failed: %s", esp_err_to_name(err));
                break;
            }

            ESP_LOGI(TAG, "frame %d", frame_count++);
            for (int row = 0; row < MLX90640_ROWS; row++) {
                char line[256];
                int pos = snprintf(line, sizeof(line), "mlx90640: row %02d:", row);
                for (int col = 0; col < MLX90640_COLS && pos < (int)sizeof(line) - 8; col++) {
                    pos += snprintf(line + pos, sizeof(line) - pos, " %.1f", matrix[row * MLX90640_COLS + col]);
                }
                ESP_LOGI(TAG, "%s", line);
            }

            vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
        }

        i2c_master_bus_rm_device(sensor.dev);
    }
}

void tire_temp_start(void)
{
    xTaskCreate(tire_temp_task, "tire_temp", 8192, NULL, 5, NULL);
}
```

- [ ] **Step 2: Commit**

```bash
git add main/tire_temp.c
git commit -m "feat: switch tire_temp task to MLX90640 matrix logging"
```

---

### Task 6: Build verification

**Files:**
- No file changes.

**Interfaces:**
- Consumes: all previous tasks.
- Produces: successful `idf.py build`.

- [ ] **Step 1: Ensure ESP-IDF environment is sourced**

```bash
. ~/.espressif/v5.5.3/esp-idf/export.sh
```

- [ ] **Step 2: Set target and build**

```bash
idf.py set-target esp32
idf.py build
```

Expected: build completes with no errors and no warnings about missing symbols.

- [ ] **Step 3: Hardware-in-the-loop check (optional but recommended)**

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Expected: UART shows lines such as:

```
I (xxxx) tire_temp: frame 0
I (xxxx) tire_temp: mlx90640: row 00: 23.4 23.5 23.4 ...
...
I (xxxx) tire_temp: mlx90640: row 23: 24.1 24.0 24.2 ...
```

- [ ] **Step 4: Commit a note if build passed**

```bash
git commit --allow-empty -m "build: verify mlx90640 integration compiles"
```

---

## Self-Review

### Spec coverage

| Spec requirement | Task(s) |
|------------------|---------|
| Add `mlx90640.c/h` driver on `i2c_master` API | Tasks 2, 3, 4 |
| Remove MLX90614 | Task 1 |
| I2C GPIO 21/22, addr 0x33, 400 kHz | Task 5 |
| 1 Hz refresh rate | Task 3 |
| Log 32×24 matrix, 24 rows, 1 decimal | Task 5 |
| Stack 8192 bytes | Task 5 |
| No MQTT matrix publishing | No code added; existing MQTT stays unused |
| `idf.py build` verification | Task 6 |

### Placeholder scan

No TBD, TODO, or vague steps. Each code block contains complete content.

### Type consistency

- `mlx90640_t` fields match usage in parameter extraction and temperature calculation.
- `MLX90640_PIXELS` is used consistently as array size.
- `mlx90640_init()` and `mlx90640_read_frame()` signatures match the header.
