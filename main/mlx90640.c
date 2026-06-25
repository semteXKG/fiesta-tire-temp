/*
 * MLX90640 thermal array driver for ESP-IDF (new i2c_master API)
 *
 * The temperature calculation algorithm is adapted from the Melexis
 * MLX90640 reference driver, used under the Apache License, Version 2.0.
 */
#include "mlx90640.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "mlx90640";

#define I2C_TIMEOUT_MS 200

/* Register addresses */
#define MLX90640_EEPROM_START_ADDRESS   0x2400
#define MLX90640_EEPROM_DUMP_NUM        832
#define MLX90640_PIXEL_DATA_START_ADDRESS 0x0400
#define MLX90640_PIXEL_NUM              768
#define MLX90640_AUX_DATA_START_ADDRESS 0x0700
#define MLX90640_AUX_NUM                64
#define MLX90640_STATUS_REG             0x8000
#define MLX90640_CTRL_REG               0x800D
#define MLX90640_INIT_STATUS_VALUE      0x0030

#define BIT_MASK(x) (1UL << (x))
#define REG_MASK(sbit, nbits) ~((~(~0UL << (nbits))) << (sbit))

#define MLX90640_CTRL_REFRESH_SHIFT     7
#define MLX90640_CTRL_REFRESH_MASK      REG_MASK(MLX90640_CTRL_REFRESH_SHIFT, 3)
#define MLX90640_CTRL_RESOLUTION_SHIFT  10
#define MLX90640_CTRL_RESOLUTION_MASK   REG_MASK(MLX90640_CTRL_RESOLUTION_SHIFT, 2)
#define MLX90640_CTRL_MEAS_MODE_SHIFT   12
#define MLX90640_CTRL_MEAS_MODE_MASK    BIT_MASK(12)
#define MLX90640_STAT_FRAME_MASK        BIT_MASK(0)
#define MLX90640_STAT_DATA_READY_MASK   BIT_MASK(3)

#define MLX90640_MS_BYTE_SHIFT          8
#define MLX90640_MS_BYTE_MASK           0xFF00
#define MLX90640_LS_BYTE_MASK           0x00FF
#define MLX90640_MS_BYTE(reg16)         (((reg16) & MLX90640_MS_BYTE_MASK) >> MLX90640_MS_BYTE_SHIFT)
#define MLX90640_LS_BYTE(reg16)         ((reg16) & MLX90640_LS_BYTE_MASK)
#define MLX90640_MSBITS_6_MASK          0xFC00
#define MLX90640_LSBITS_10_MASK         0x03FF
#define MLX90640_NIBBLE1_MASK           0x000F
#define MLX90640_NIBBLE2_MASK           0x00F0
#define MLX90640_NIBBLE3_MASK           0x0F00
#define MLX90640_NIBBLE4_MASK           0xF000
#define MLX90640_NIBBLE1(reg16)         ((reg16) & MLX90640_NIBBLE1_MASK)
#define MLX90640_NIBBLE2(reg16)         (((reg16) & MLX90640_NIBBLE2_MASK) >> 4)
#define MLX90640_NIBBLE3(reg16)         (((reg16) & MLX90640_NIBBLE3_MASK) >> 8)
#define MLX90640_NIBBLE4(reg16)         (((reg16) & MLX90640_NIBBLE4_MASK) >> 12)

#define POW2(x)                         powf(2.0f, (float)(x))
#define SCALEALPHA                      0.000001f

/* Internal helpers */
static esp_err_t mlx90640_i2c_read(mlx90640_t *s, uint16_t start_addr, uint16_t nwords, uint16_t *data);
static esp_err_t mlx90640_i2c_write(mlx90640_t *s, uint16_t write_addr, uint16_t data);
static int mlx90640_extract_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640);
static void mlx90640_calculate_to(uint16_t *frameData, const mlx90640_params_t *params, float emissivity, float tr, float *result);
static float mlx90640_get_vdd(uint16_t *frameData, const mlx90640_params_t *params);
static float mlx90640_get_ta(uint16_t *frameData, const mlx90640_params_t *params);
static int mlx90640_is_pixel_bad(uint16_t pixel, const mlx90640_params_t *params);
static void mlx90640_bad_pixels_correction(uint16_t *pixels, float *to, int mode, mlx90640_params_t *params);

/*------------------------ Public API ------------------------*/

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

    static uint16_t eeData[MLX90640_EEPROM_DUMP_NUM];
    err = mlx90640_i2c_read(out, MLX90640_EEPROM_START_ADDRESS, MLX90640_EEPROM_DUMP_NUM, eeData);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "EEPROM read failed: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(out->dev);
        return err;
    }

    /* Minimal EEPROM sanity check */
    if (eeData[0x0A] == 0x0000 || eeData[0x0A] == 0xFFFF) {
        ESP_LOGE(TAG, "EEPROM sanity check failed (word 0x0A = 0x%04X)", eeData[0x0A]);
        i2c_master_bus_rm_device(out->dev);
        return ESP_ERR_INVALID_RESPONSE;
    }

    int warn = mlx90640_extract_parameters(eeData, &out->params);
    if (warn != 0) {
        ESP_LOGW(TAG, "EEPROM deviating-pixel warning: %d", warn);
    }

    /* Set refresh rate to 1 Hz */
    uint16_t ctrl;
    err = mlx90640_i2c_read(out, MLX90640_CTRL_REG, 1, &ctrl);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(out->dev);
        return err;
    }
    ctrl &= MLX90640_CTRL_REFRESH_MASK;
    ctrl |= ((uint16_t)1 << MLX90640_CTRL_REFRESH_SHIFT);
    err = mlx90640_i2c_write(out, MLX90640_CTRL_REG, ctrl);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(out->dev);
        return err;
    }

    ESP_LOGI(TAG, "MLX90640 initialized @ 0x%02X", addr);
    return ESP_OK;
}

esp_err_t mlx90640_read_frame(mlx90640_t *s, float out_temps[MLX90640_PIXELS],
                               float emissivity, float reflected_temp_c)
{
    if (s == NULL || out_temps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (emissivity <= 0.0f || emissivity > 1.0f || reflected_temp_c < -100.0f || reflected_temp_c > 200.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    static uint16_t frameData[MLX90640_PIXEL_NUM + MLX90640_AUX_NUM + 2];
    uint16_t statusRegister;
    uint16_t controlRegister1;
    esp_err_t err;

    memset(out_temps, 0, MLX90640_PIXELS * sizeof(float));

    int first_subpage = -1;
    uint8_t frame_mode = 0;

    /* Read both subpages to fill the full 32x24 matrix */
    for (int subpage_read = 0; subpage_read < 2; subpage_read++) {
        /* Wait for new data (data-ready bit set) */
        bool ready = false;
        for (int attempt = 0; attempt < 100; attempt++) {
            err = mlx90640_i2c_read(s, MLX90640_STATUS_REG, 1, &statusRegister);
            if (err != ESP_OK) {
                return err;
            }
            if (statusRegister & MLX90640_STAT_DATA_READY_MASK) {
                ready = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (!ready) {
            ESP_LOGE(TAG, "data-ready timeout");
            return ESP_ERR_TIMEOUT;
        }

        /* Read pixel and auxiliary data */
        err = mlx90640_i2c_read(s, MLX90640_PIXEL_DATA_START_ADDRESS, MLX90640_PIXEL_NUM, frameData);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "pixel data read failed: %s", esp_err_to_name(err));
            return err;
        }

        err = mlx90640_i2c_read(s, MLX90640_AUX_DATA_START_ADDRESS, MLX90640_AUX_NUM,
                                &frameData[MLX90640_PIXEL_NUM]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "aux data read failed: %s", esp_err_to_name(err));
            return err;
        }

        err = mlx90640_i2c_read(s, MLX90640_CTRL_REG, 1, &controlRegister1);
        if (err != ESP_OK) {
            return err;
        }
        frameData[MLX90640_PIXEL_NUM + MLX90640_AUX_NUM] = controlRegister1;
        frame_mode = (uint8_t)((controlRegister1 & MLX90640_CTRL_MEAS_MODE_MASK) >> MLX90640_CTRL_MEAS_MODE_SHIFT);
        uint16_t subpage = statusRegister & MLX90640_STAT_FRAME_MASK;
        frameData[MLX90640_PIXEL_NUM + MLX90640_AUX_NUM + 1] = subpage;

        if (first_subpage == -1) {
            first_subpage = (int)subpage;
        } else if ((int)subpage == first_subpage) {
            ESP_LOGE(TAG, "subpage did not advance (stuck at %d)", first_subpage);
            return ESP_ERR_INVALID_RESPONSE;
        }

        /* Clear new-data bit after reading the frame */
        err = mlx90640_i2c_write(s, MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE);
        if (err != ESP_OK) {
            return err;
        }

        mlx90640_calculate_to(frameData, &s->params, emissivity, reflected_temp_c, out_temps);
    }

    /* Replace deviating pixels with neighbor averages */
    mlx90640_bad_pixels_correction(s->params.brokenPixels, out_temps, frame_mode, &s->params);
    mlx90640_bad_pixels_correction(s->params.outlierPixels, out_temps, frame_mode, &s->params);

    return ESP_OK;
}

/*------------------------ I2C wrapper ------------------------*/

static esp_err_t mlx90640_i2c_read(mlx90640_t *s, uint16_t start_addr, uint16_t nwords, uint16_t *data)
{
    uint8_t cmd[2] = { (uint8_t)(start_addr >> 8), (uint8_t)(start_addr & 0xFF) };
    size_t len = nwords * 2;

    esp_err_t err = i2c_master_transmit_receive(s->dev, cmd, sizeof(cmd),
                                                (uint8_t *)data, len, I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        /* Sensor sends big-endian 16-bit words; swap in place for little-endian host. */
        for (size_t i = 0; i < nwords; i++) {
            data[i] = (data[i] << 8) | (data[i] >> 8);
        }
    }
    return err;
}

static esp_err_t mlx90640_i2c_write(mlx90640_t *s, uint16_t write_addr, uint16_t data)
{
    uint8_t tx[4] = { (uint8_t)(write_addr >> 8), (uint8_t)(write_addr & 0xFF),
                      (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return i2c_master_transmit(s->dev, tx, sizeof(tx), I2C_TIMEOUT_MS);
}

/*------------------------ Parameter extraction (Melexis) ------------------------*/

static void mlx90640_extract_vdd_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    int8_t kVdd = (int8_t)MLX90640_MS_BYTE(eeData[51]);
    int16_t vdd25 = MLX90640_LS_BYTE(eeData[51]);
    vdd25 = (int16_t)(((uint16_t)(vdd25 - 256) << 5) - 8192);
    mlx90640->kVdd = 32 * kVdd;
    mlx90640->vdd25 = vdd25;
}

static void mlx90640_extract_ptat_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    float KvPTAT = (eeData[50] & MLX90640_MSBITS_6_MASK) >> 10;
    if (KvPTAT > 31.0f) {
        KvPTAT -= 64.0f;
    }
    KvPTAT /= 4096.0f;

    float KtPTAT = eeData[50] & MLX90640_LSBITS_10_MASK;
    if (KtPTAT > 511.0f) {
        KtPTAT -= 1024.0f;
    }
    KtPTAT /= 8.0f;

    mlx90640->KvPTAT = KvPTAT;
    mlx90640->KtPTAT = KtPTAT;
    mlx90640->vPTAT25 = eeData[49];
    mlx90640->alphaPTAT = (float)((eeData[16] & MLX90640_NIBBLE4_MASK) >> 12) / POW2(14) + 8.0f;
}

static void mlx90640_extract_gain_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    mlx90640->gainEE = (int16_t)eeData[48];
}

static void mlx90640_extract_tgc_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    mlx90640->tgc = (int8_t)MLX90640_LS_BYTE(eeData[60]) / 32.0f;
}

static void mlx90640_extract_resolution_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    mlx90640->resolutionEE = (uint8_t)((eeData[56] & 0x3000) >> 12);
}

static void mlx90640_extract_ks_ta_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    mlx90640->KsTa = (int8_t)MLX90640_MS_BYTE(eeData[60]) / 8192.0f;
}

static void mlx90640_extract_ks_to_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    int32_t KsToScale = MLX90640_NIBBLE1(eeData[63]) + 8;
    KsToScale = 1UL << KsToScale;
    int8_t step = (int8_t)(((eeData[63] & 0x3000) >> 12) * 10);

    mlx90640->ct[0] = -40;
    mlx90640->ct[1] = 0;
    mlx90640->ct[2] = (int16_t)MLX90640_NIBBLE2(eeData[63]) * step;
    mlx90640->ct[3] = mlx90640->ct[2] + (int16_t)MLX90640_NIBBLE3(eeData[63]) * step;
    mlx90640->ct[4] = 400;

    mlx90640->ksTo[0] = (int8_t)MLX90640_LS_BYTE(eeData[61]) / (float)KsToScale;
    mlx90640->ksTo[1] = (int8_t)MLX90640_MS_BYTE(eeData[61]) / (float)KsToScale;
    mlx90640->ksTo[2] = (int8_t)MLX90640_LS_BYTE(eeData[62]) / (float)KsToScale;
    mlx90640->ksTo[3] = (int8_t)MLX90640_MS_BYTE(eeData[62]) / (float)KsToScale;
    mlx90640->ksTo[4] = -0.0002f;
}

static void mlx90640_extract_alpha_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    int accRow[24];
    int accColumn[32];
    int p = 0;
    int alphaRef;
    uint8_t alphaScale;
    uint8_t accRowScale;
    uint8_t accColumnScale;
    uint8_t accRemScale;
    float alphaTemp[MLX90640_PIXELS];
    float temp;

    accRemScale = (uint8_t)MLX90640_NIBBLE1(eeData[32]);
    accColumnScale = (uint8_t)MLX90640_NIBBLE2(eeData[32]);
    accRowScale = (uint8_t)MLX90640_NIBBLE3(eeData[32]);
    alphaScale = (uint8_t)MLX90640_NIBBLE4(eeData[32]) + 30;
    alphaRef = (int)eeData[33];

    for (int i = 0; i < 6; i++) {
        p = i * 4;
        accRow[p + 0] = (int)MLX90640_NIBBLE1(eeData[34 + i]);
        accRow[p + 1] = (int)MLX90640_NIBBLE2(eeData[34 + i]);
        accRow[p + 2] = (int)MLX90640_NIBBLE3(eeData[34 + i]);
        accRow[p + 3] = (int)MLX90640_NIBBLE4(eeData[34 + i]);
    }

    for (int i = 0; i < MLX90640_ROWS; i++) {
        if (accRow[i] > 7) {
            accRow[i] -= 16;
        }
    }

    for (int i = 0; i < 8; i++) {
        p = i * 4;
        accColumn[p + 0] = (int)MLX90640_NIBBLE1(eeData[40 + i]);
        accColumn[p + 1] = (int)MLX90640_NIBBLE2(eeData[40 + i]);
        accColumn[p + 2] = (int)MLX90640_NIBBLE3(eeData[40 + i]);
        accColumn[p + 3] = (int)MLX90640_NIBBLE4(eeData[40 + i]);
    }

    for (int i = 0; i < MLX90640_COLS; i++) {
        if (accColumn[i] > 7) {
            accColumn[i] -= 16;
        }
    }

    for (int i = 0; i < MLX90640_ROWS; i++) {
        for (int j = 0; j < MLX90640_COLS; j++) {
            p = 32 * i + j;
            alphaTemp[p] = (float)((eeData[64 + p] & 0x03F0) >> 4);
            if (alphaTemp[p] > 31.0f) {
                alphaTemp[p] -= 64.0f;
            }
            alphaTemp[p] *= (float)(1 << accRemScale);
            alphaTemp[p] = (float)(alphaRef + (accRow[i] << accRowScale) +
                                   (accColumn[j] << accColumnScale) + (int)alphaTemp[p]);
            alphaTemp[p] /= POW2(alphaScale);
            alphaTemp[p] -= mlx90640->tgc *
                             (mlx90640->cpAlpha[0] + mlx90640->cpAlpha[1]) / 2.0f;
            alphaTemp[p] = SCALEALPHA / alphaTemp[p];
        }
    }

    temp = alphaTemp[0];
    for (int i = 1; i < MLX90640_PIXELS; i++) {
        if (alphaTemp[i] > temp) {
            temp = alphaTemp[i];
        }
    }

    alphaScale = 0;
    while (temp < 32767.4f) {
        temp *= 2.0f;
        alphaScale++;
    }

    for (int i = 0; i < MLX90640_PIXELS; i++) {
        temp = alphaTemp[i] * POW2(alphaScale);
        mlx90640->alpha[i] = (uint16_t)(temp + 0.5f);
    }

    mlx90640->alphaScale = alphaScale;
}

static void mlx90640_extract_offset_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    int occRow[24];
    int occColumn[32];
    int p = 0;
    int16_t offsetRef;
    uint8_t occRowScale;
    uint8_t occColumnScale;
    uint8_t occRemScale;

    occRemScale = (uint8_t)MLX90640_NIBBLE1(eeData[16]);
    occColumnScale = (uint8_t)MLX90640_NIBBLE2(eeData[16]);
    occRowScale = (uint8_t)MLX90640_NIBBLE3(eeData[16]);
    offsetRef = (int16_t)eeData[17];

    for (int i = 0; i < 6; i++) {
        p = i * 4;
        occRow[p + 0] = (int)MLX90640_NIBBLE1(eeData[18 + i]);
        occRow[p + 1] = (int)MLX90640_NIBBLE2(eeData[18 + i]);
        occRow[p + 2] = (int)MLX90640_NIBBLE3(eeData[18 + i]);
        occRow[p + 3] = (int)MLX90640_NIBBLE4(eeData[18 + i]);
    }

    for (int i = 0; i < MLX90640_ROWS; i++) {
        if (occRow[i] > 7) {
            occRow[i] -= 16;
        }
    }

    for (int i = 0; i < 8; i++) {
        p = i * 4;
        occColumn[p + 0] = (int)MLX90640_NIBBLE1(eeData[24 + i]);
        occColumn[p + 1] = (int)MLX90640_NIBBLE2(eeData[24 + i]);
        occColumn[p + 2] = (int)MLX90640_NIBBLE3(eeData[24 + i]);
        occColumn[p + 3] = (int)MLX90640_NIBBLE4(eeData[24 + i]);
    }

    for (int i = 0; i < MLX90640_COLS; i++) {
        if (occColumn[i] > 7) {
            occColumn[i] -= 16;
        }
    }

    for (int i = 0; i < MLX90640_ROWS; i++) {
        for (int j = 0; j < MLX90640_COLS; j++) {
            p = 32 * i + j;
            mlx90640->offset[p] = (int16_t)((eeData[64 + p] & MLX90640_MSBITS_6_MASK) >> 10);
            if (mlx90640->offset[p] > 31) {
                mlx90640->offset[p] -= 64;
            }
            mlx90640->offset[p] = (int16_t)(mlx90640->offset[p] * (1 << occRemScale));
            mlx90640->offset[p] = (int16_t)(offsetRef + (occRow[i] << occRowScale) +
                                              (occColumn[j] << occColumnScale) + mlx90640->offset[p]);
        }
    }
}

static void mlx90640_extract_kta_pixel_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    int8_t KtaRC[4];
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    uint8_t split;
    float ktaTemp[MLX90640_PIXELS];
    float temp;

    KtaRC[0] = (int8_t)MLX90640_MS_BYTE(eeData[54]);
    KtaRC[2] = (int8_t)MLX90640_LS_BYTE(eeData[54]);
    KtaRC[1] = (int8_t)MLX90640_MS_BYTE(eeData[55]);
    KtaRC[3] = (int8_t)MLX90640_LS_BYTE(eeData[55]);

    ktaScale1 = (uint8_t)MLX90640_NIBBLE2(eeData[56]) + 8;
    ktaScale2 = (uint8_t)MLX90640_NIBBLE1(eeData[56]);

    for (int i = 0; i < MLX90640_ROWS; i++) {
        for (int j = 0; j < MLX90640_COLS; j++) {
            int p = 32 * i + j;
            split = (uint8_t)(2 * (p / 32 - (p / 64) * 2) + p % 2);
            ktaTemp[p] = (float)((eeData[64 + p] & 0x000E) >> 1);
            if (ktaTemp[p] > 3.0f) {
                ktaTemp[p] -= 8.0f;
            }
            ktaTemp[p] *= (float)(1 << ktaScale2);
            ktaTemp[p] = (float)KtaRC[split] + ktaTemp[p];
            ktaTemp[p] /= POW2(ktaScale1);
        }
    }

    temp = fabsf(ktaTemp[0]);
    for (int i = 1; i < MLX90640_PIXELS; i++) {
        if (fabsf(ktaTemp[i]) > temp) {
            temp = fabsf(ktaTemp[i]);
        }
    }

    ktaScale1 = 0;
    while (temp < 63.4f) {
        temp *= 2.0f;
        ktaScale1++;
    }

    for (int i = 0; i < MLX90640_PIXELS; i++) {
        temp = ktaTemp[i] * POW2(ktaScale1);
        if (temp < 0.0f) {
            mlx90640->kta[i] = (int8_t)(temp - 0.5f);
        } else {
            mlx90640->kta[i] = (int8_t)(temp + 0.5f);
        }
    }

    mlx90640->ktaScale = ktaScale1;
}

static void mlx90640_extract_kv_pixel_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    int8_t KvT[4];
    int8_t KvRoCo;
    int8_t KvReCo;
    int8_t KvRoCe;
    int8_t KvReCe;
    uint8_t kvScale;
    uint8_t split;
    float kvTemp[MLX90640_PIXELS];
    float temp;

    KvRoCo = (int8_t)MLX90640_NIBBLE4(eeData[52]);
    if (KvRoCo > 7) KvRoCo -= 16;
    KvT[0] = KvRoCo;

    KvReCo = (int8_t)MLX90640_NIBBLE3(eeData[52]);
    if (KvReCo > 7) KvReCo -= 16;
    KvT[2] = KvReCo;

    KvRoCe = (int8_t)MLX90640_NIBBLE2(eeData[52]);
    if (KvRoCe > 7) KvRoCe -= 16;
    KvT[1] = KvRoCe;

    KvReCe = (int8_t)MLX90640_NIBBLE1(eeData[52]);
    if (KvReCe > 7) KvReCe -= 16;
    KvT[3] = KvReCe;

    kvScale = (uint8_t)MLX90640_NIBBLE3(eeData[56]);

    for (int i = 0; i < MLX90640_ROWS; i++) {
        for (int j = 0; j < MLX90640_COLS; j++) {
            int p = 32 * i + j;
            split = (uint8_t)(2 * (p / 32 - (p / 64) * 2) + p % 2);
            kvTemp[p] = (float)KvT[split];
            kvTemp[p] /= POW2(kvScale);
        }
    }

    temp = fabsf(kvTemp[0]);
    for (int i = 1; i < MLX90640_PIXELS; i++) {
        if (fabsf(kvTemp[i]) > temp) {
            temp = fabsf(kvTemp[i]);
        }
    }

    kvScale = 0;
    while (temp < 63.4f) {
        temp *= 2.0f;
        kvScale++;
    }

    for (int i = 0; i < MLX90640_PIXELS; i++) {
        temp = kvTemp[i] * POW2(kvScale);
        if (temp < 0.0f) {
            mlx90640->kv[i] = (int8_t)(temp - 0.5f);
        } else {
            mlx90640->kv[i] = (int8_t)(temp + 0.5f);
        }
    }

    mlx90640->kvScale = kvScale;
}

static void mlx90640_extract_cp_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    float alphaSP[2];
    int16_t offsetSP[2];
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;

    alphaScale = (uint8_t)MLX90640_NIBBLE4(eeData[32]) + 27;

    offsetSP[0] = (int16_t)(eeData[58] & MLX90640_LSBITS_10_MASK);
    if (offsetSP[0] > 511) {
        offsetSP[0] -= 1024;
    }

    offsetSP[1] = (int16_t)((eeData[58] & MLX90640_MSBITS_6_MASK) >> 10);
    if (offsetSP[1] > 31) {
        offsetSP[1] -= 64;
    }
    offsetSP[1] = (int16_t)(offsetSP[1] + offsetSP[0]);

    alphaSP[0] = (float)(eeData[57] & MLX90640_LSBITS_10_MASK);
    if (alphaSP[0] > 511.0f) {
        alphaSP[0] -= 1024.0f;
    }
    alphaSP[0] /= POW2(alphaScale);

    alphaSP[1] = (float)((eeData[57] & MLX90640_MSBITS_6_MASK) >> 10);
    if (alphaSP[1] > 31.0f) {
        alphaSP[1] -= 64.0f;
    }
    alphaSP[1] = (1.0f + alphaSP[1] / 128.0f) * alphaSP[0];

    cpKta = (float)(int8_t)MLX90640_LS_BYTE(eeData[59]);
    ktaScale1 = (uint8_t)MLX90640_NIBBLE2(eeData[56]) + 8;
    mlx90640->cpKta = cpKta / POW2(ktaScale1);

    cpKv = (float)(int8_t)MLX90640_MS_BYTE(eeData[59]);
    kvScale = (uint8_t)MLX90640_NIBBLE3(eeData[56]);
    mlx90640->cpKv = cpKv / POW2(kvScale);

    mlx90640->cpAlpha[0] = alphaSP[0];
    mlx90640->cpAlpha[1] = alphaSP[1];
    mlx90640->cpOffset[0] = offsetSP[0];
    mlx90640->cpOffset[1] = offsetSP[1];
}

static void mlx90640_extract_cilc_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    float ilChessC[3];
    uint8_t calibrationModeEE;

    calibrationModeEE = (uint8_t)((eeData[10] & 0x0800) >> 11);

    ilChessC[0] = (float)(eeData[53] & 0x003F);
    if (ilChessC[0] > 31.0f) {
        ilChessC[0] -= 64.0f;
    }
    ilChessC[0] /= 16.0f;

    ilChessC[1] = (float)((eeData[53] & 0x07C0) >> 6);
    if (ilChessC[1] > 15.0f) {
        ilChessC[1] -= 32.0f;
    }
    ilChessC[1] /= 2.0f;

    ilChessC[2] = (float)((eeData[53] & 0xF800) >> 11);
    if (ilChessC[2] > 15.0f) {
        ilChessC[2] -= 32.0f;
    }
    ilChessC[2] /= 8.0f;

    mlx90640->calibrationModeEE = calibrationModeEE;
    mlx90640->ilChessC[0] = ilChessC[0];
    mlx90640->ilChessC[1] = ilChessC[1];
    mlx90640->ilChessC[2] = ilChessC[2];
}

static int mlx90640_check_adjacent_pixels(uint16_t pix1, uint16_t pix2)
{
    uint16_t lp1 = pix1 >> 5;
    uint16_t lp2 = pix2 >> 5;
    uint16_t cp1 = pix1 - (lp1 << 5);
    uint16_t cp2 = pix2 - (lp2 << 5);

    int pixPosDif = (int)lp1 - (int)lp2;
    if (pixPosDif > -2 && pixPosDif < 2) {
        pixPosDif = (int)cp1 - (int)cp2;
        if (pixPosDif > -2 && pixPosDif < 2) {
            return -6;
        }
    }
    return 0;
}

static int mlx90640_extract_deviating_pixels(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;
    uint16_t outlierPixCnt = 0;
    int warn = 0;

    for (pixCnt = 0; pixCnt < 5; pixCnt++) {
        mlx90640->brokenPixels[pixCnt] = 0xFFFF;
        mlx90640->outlierPixels[pixCnt] = 0xFFFF;
    }

    pixCnt = 0;
    while (pixCnt < MLX90640_PIXELS && brokenPixCnt < 5 && outlierPixCnt < 5) {
        if (eeData[pixCnt + 64] == 0) {
            mlx90640->brokenPixels[brokenPixCnt] = pixCnt;
            brokenPixCnt++;
        } else if ((eeData[pixCnt + 64] & 0x0001) != 0) {
            mlx90640->outlierPixels[outlierPixCnt] = pixCnt;
            outlierPixCnt++;
        }
        pixCnt++;
    }

    if (brokenPixCnt > 4) {
        warn = -3;
    } else if (outlierPixCnt > 4) {
        warn = -4;
    } else if ((brokenPixCnt + outlierPixCnt) > 4) {
        warn = -5;
    } else {
        for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++) {
            for (int i = pixCnt + 1; i < brokenPixCnt; i++) {
                warn = mlx90640_check_adjacent_pixels(mlx90640->brokenPixels[pixCnt],
                                                       mlx90640->brokenPixels[i]);
                if (warn != 0) {
                    return warn;
                }
            }
        }

        for (pixCnt = 0; pixCnt < outlierPixCnt; pixCnt++) {
            for (int i = pixCnt + 1; i < outlierPixCnt; i++) {
                warn = mlx90640_check_adjacent_pixels(mlx90640->outlierPixels[pixCnt],
                                                       mlx90640->outlierPixels[i]);
                if (warn != 0) {
                    return warn;
                }
            }
        }

        for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++) {
            for (int i = 0; i < outlierPixCnt; i++) {
                warn = mlx90640_check_adjacent_pixels(mlx90640->brokenPixels[pixCnt],
                                                       mlx90640->outlierPixels[i]);
                if (warn != 0) {
                    return warn;
                }
            }
        }
    }

    return warn;
}

static int mlx90640_extract_parameters(uint16_t *eeData, mlx90640_params_t *mlx90640)
{
    mlx90640_extract_vdd_parameters(eeData, mlx90640);
    mlx90640_extract_ptat_parameters(eeData, mlx90640);
    mlx90640_extract_gain_parameters(eeData, mlx90640);
    mlx90640_extract_tgc_parameters(eeData, mlx90640);
    mlx90640_extract_resolution_parameters(eeData, mlx90640);
    mlx90640_extract_ks_ta_parameters(eeData, mlx90640);
    mlx90640_extract_ks_to_parameters(eeData, mlx90640);
    mlx90640_extract_cp_parameters(eeData, mlx90640);
    mlx90640_extract_alpha_parameters(eeData, mlx90640);
    mlx90640_extract_offset_parameters(eeData, mlx90640);
    mlx90640_extract_kta_pixel_parameters(eeData, mlx90640);
    mlx90640_extract_kv_pixel_parameters(eeData, mlx90640);
    mlx90640_extract_cilc_parameters(eeData, mlx90640);
    return mlx90640_extract_deviating_pixels(eeData, mlx90640);
}

static int mlx90640_is_pixel_bad(uint16_t pixel, const mlx90640_params_t *params)
{
    for (int i = 0; i < 5; i++) {
        if (pixel == params->outlierPixels[i] || pixel == params->brokenPixels[i]) {
            return 1;
        }
    }
    return 0;
}

static void mlx90640_bad_pixels_correction(uint16_t *pixels, float *to, int mode, mlx90640_params_t *params)
{
    float ap[4];
    uint16_t pix = 0;

    while (pixels[pix] != 0xFFFF) {
        uint8_t line = pixels[pix] >> 5;
        uint8_t column = (uint8_t)(pixels[pix] - (line << 5));

        if (mode == 1) {
            if (line == 0) {
                if (column == 0) {
                    to[pixels[pix]] = to[33];
                } else if (column == 31) {
                    to[pixels[pix]] = to[62];
                } else {
                    to[pixels[pix]] = (to[pixels[pix] + 31] + to[pixels[pix] + 33]) / 2.0f;
                }
            } else if (line == 23) {
                if (column == 0) {
                    to[pixels[pix]] = to[705];
                } else if (column == 31) {
                    to[pixels[pix]] = to[734];
                } else {
                    to[pixels[pix]] = (to[pixels[pix] - 33] + to[pixels[pix] - 31]) / 2.0f;
                }
            } else if (column == 0) {
                to[pixels[pix]] = (to[pixels[pix] - 31] + to[pixels[pix] + 33]) / 2.0f;
            } else if (column == 31) {
                to[pixels[pix]] = (to[pixels[pix] - 33] + to[pixels[pix] + 31]) / 2.0f;
            } else {
                ap[0] = to[pixels[pix] - 33];
                ap[1] = to[pixels[pix] - 31];
                ap[2] = to[pixels[pix] + 31];
                ap[3] = to[pixels[pix] + 33];

                /* simple median of four */
                for (int i = 0; i < 3; i++) {
                    for (int j = i + 1; j < 4; j++) {
                        if (ap[j] < ap[i]) {
                            float tmp = ap[i];
                            ap[i] = ap[j];
                            ap[j] = tmp;
                        }
                    }
                }
                to[pixels[pix]] = (ap[1] + ap[2]) / 2.0f;
            }
        } else {
            if (column == 0) {
                to[pixels[pix]] = to[pixels[pix] + 1];
            } else if (column == 1 || column == 30) {
                to[pixels[pix]] = (to[pixels[pix] - 1] + to[pixels[pix] + 1]) / 2.0f;
            } else if (column == 31) {
                to[pixels[pix]] = to[pixels[pix] - 1];
            } else {
                if (mlx90640_is_pixel_bad(pixels[pix] - 2, params) == 0 &&
                    mlx90640_is_pixel_bad(pixels[pix] + 2, params) == 0) {
                    ap[0] = to[pixels[pix] + 1] - to[pixels[pix] + 2];
                    ap[1] = to[pixels[pix] - 1] - to[pixels[pix] - 2];
                    if (fabsf(ap[0]) > fabsf(ap[1])) {
                        to[pixels[pix]] = to[pixels[pix] - 1] + ap[1];
                    } else {
                        to[pixels[pix]] = to[pixels[pix] + 1] + ap[0];
                    }
                } else {
                    to[pixels[pix]] = (to[pixels[pix] - 1] + to[pixels[pix] + 1]) / 2.0f;
                }
            }
        }
        pix++;
    }
}

/*------------------------ Temperature calculation (Melexis) ------------------------*/

static float mlx90640_get_vdd(uint16_t *frameData, const mlx90640_params_t *params)
{
    uint16_t resolutionRAM = (frameData[832] & ~MLX90640_CTRL_RESOLUTION_MASK) >> MLX90640_CTRL_RESOLUTION_SHIFT;
    float resolutionCorrection = POW2(params->resolutionEE) / POW2(resolutionRAM);
    return (resolutionCorrection * (int16_t)frameData[810] - params->vdd25) / params->kVdd + 3.3f;
}

static float mlx90640_get_ta(uint16_t *frameData, const mlx90640_params_t *params)
{
    float vdd = mlx90640_get_vdd(frameData, params);
    int16_t ptat = (int16_t)frameData[800];
    float ptatArt = (ptat / (ptat * params->alphaPTAT + (int16_t)frameData[768])) * POW2(18);
    float ta = (ptatArt / (1.0f + params->KvPTAT * (vdd - 3.3f)) - params->vPTAT25);
    ta = ta / params->KtPTAT + 25.0f;
    return ta;
}

static void mlx90640_calculate_to(uint16_t *frameData, const mlx90640_params_t *params,
                                   float emissivity, float tr, float *result)
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
    uint8_t mode;
    int8_t ilPattern;
    int8_t chessPattern;
    int8_t pattern;
    int8_t conversionPattern;
    float Sx;
    float To;
    float alphaCorrR[4];
    int8_t range;
    uint16_t subPage;
    float ktaScale;
    float kvScale;
    float alphaScale;
    float kta;
    float kv;

    subPage = frameData[833];
    vdd = mlx90640_get_vdd(frameData, params);
    ta = mlx90640_get_ta(frameData, params);

    ta4 = ta + 273.15f;
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    tr4 = tr + 273.15f;
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    taTr = tr4 - (tr4 - ta4) / emissivity;

    ktaScale = POW2(params->ktaScale);
    kvScale = POW2(params->kvScale);
    alphaScale = POW2(params->alphaScale);

    alphaCorrR[0] = 1.0f / (1.0f + params->ksTo[0] * 40.0f);
    alphaCorrR[1] = 1.0f;
    alphaCorrR[2] = (1.0f + params->ksTo[1] * params->ct[2]);
    alphaCorrR[3] = alphaCorrR[2] * (1.0f + params->ksTo[2] * (params->ct[3] - params->ct[2]));

    gain = (float)params->gainEE / (int16_t)frameData[778];

    mode = (frameData[832] & MLX90640_CTRL_MEAS_MODE_MASK) >> MLX90640_CTRL_MEAS_MODE_SHIFT;

    irDataCP[0] = (int16_t)frameData[776] * gain;
    irDataCP[1] = (int16_t)frameData[808] * gain;

    irDataCP[0] = irDataCP[0] - params->cpOffset[0] *
                    (1.0f + params->cpKta * (ta - 25.0f)) *
                    (1.0f + params->cpKv * (vdd - 3.3f));

    if (mode == params->calibrationModeEE) {
        irDataCP[1] = irDataCP[1] - params->cpOffset[1] *
                        (1.0f + params->cpKta * (ta - 25.0f)) *
                        (1.0f + params->cpKv * (vdd - 3.3f));
    } else {
        irDataCP[1] = irDataCP[1] - (params->cpOffset[1] + params->ilChessC[0]) *
                        (1.0f + params->cpKta * (ta - 25.0f)) *
                        (1.0f + params->cpKv * (vdd - 3.3f));
    }

    for (int pixelNumber = 0; pixelNumber < MLX90640_PIXELS; pixelNumber++) {
        ilPattern = (int8_t)(pixelNumber / 32 - (pixelNumber / 64) * 2);
        chessPattern = (int8_t)(ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2));
        conversionPattern = (int8_t)(((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 +
                                       (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern));

        if (mode == 0) {
            pattern = ilPattern;
        } else {
            pattern = chessPattern;
        }

        if (pattern == frameData[833]) {
            irData = (int16_t)frameData[pixelNumber] * gain;

            kta = params->kta[pixelNumber] / ktaScale;
            kv = params->kv[pixelNumber] / kvScale;
            irData = irData - params->offset[pixelNumber] *
                     (1.0f + kta * (ta - 25.0f)) *
                     (1.0f + kv * (vdd - 3.3f));

            if (mode != params->calibrationModeEE) {
                irData = irData + params->ilChessC[2] * (2 * ilPattern - 1) -
                         params->ilChessC[1] * conversionPattern;
            }

            irData = irData - params->tgc * irDataCP[subPage];
            irData = irData / emissivity;

            alphaCompensated = SCALEALPHA * alphaScale / params->alpha[pixelNumber];
            alphaCompensated = alphaCompensated * (1.0f + params->KsTa * (ta - 25.0f));

            Sx = alphaCompensated * alphaCompensated * alphaCompensated *
                 (irData + alphaCompensated * taTr);
            Sx = sqrtf(sqrtf(Sx)) * params->ksTo[1];

            To = sqrtf(sqrtf(irData / (alphaCompensated * (1.0f - params->ksTo[1] * 273.15f) + Sx) + taTr)) - 273.15f;

            if (To < params->ct[1]) {
                range = 0;
            } else if (To < params->ct[2]) {
                range = 1;
            } else if (To < params->ct[3]) {
                range = 2;
            } else {
                range = 3;
            }

            To = sqrtf(sqrtf(irData / (alphaCompensated * alphaCorrR[range] *
                              (1.0f + params->ksTo[range] * (To - params->ct[range]))) + taTr)) - 273.15f;

            result[pixelNumber] = To;
        }
    }
}
