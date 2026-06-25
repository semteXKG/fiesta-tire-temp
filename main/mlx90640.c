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
    p->vdd25 = ((int16_t)(p->vdd25 - 256) * 32) - 8192;

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

    {
        uint16_t gain_raw = eeData[48];
        p->gainEE = (gain_raw > 32767) ? (int16_t)(gain_raw - 65536) : (int16_t)gain_raw;
    }

    p->tgc = eeData[60] & 0x00FF;
    if (p->tgc > 127) {
        p->tgc = p->tgc - 256;
    }
    p->tgc = p->tgc / 32.0f;

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
