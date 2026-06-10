/*
 * MLX90614 SMBus driver for ESP-IDF (new i2c_master API)
 *
 * Protocol notes:
 *  - SMBus "Read Word": S | addr+W | command | Sr | addr+R | LSB | MSB | PEC | P
 *  - SMBus "Write Word": S | addr+W | command | LSB | MSB | PEC | P
 *  - PEC is CRC-8 (poly 0x07) over ALL bytes on the wire including
 *    both address bytes.
 *  - Max bus speed 100 kHz. The sensor needs >= 1.44 ms between the
 *    end of one access and the start of the next.
 *  - Temperature: raw * 0.02 K - 273.15 = degrees C
 */
#include "mlx90614.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "mlx90614";

#define I2C_TIMEOUT_MS 50
#define EEPROM_WRITE_DELAY_MS 10   /* datasheet: 5 ms typ. erase/write */

/* CRC-8, polynomial x^8 + x^2 + x + 1 (0x07), init 0x00 */
static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

esp_err_t mlx90614_init(i2c_master_bus_handle_t bus, uint8_t addr, mlx90614_t *out)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000,   /* MLX90614 max is 100 kHz */
    };
    out->addr = addr;
    return i2c_master_bus_add_device(bus, &cfg, &out->dev);
}

esp_err_t mlx90614_read_word(mlx90614_t *s, uint8_t command, uint16_t *value)
{
    uint8_t rx[3]; /* LSB, MSB, PEC */
    esp_err_t err = i2c_master_transmit_receive(s->dev, &command, 1,
                                                rx, sizeof(rx),
                                                I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    /* Verify PEC over: addr+W, command, addr+R, LSB, MSB */
    uint8_t frame[5] = {
        (uint8_t)(s->addr << 1),        /* write address */
        command,
        (uint8_t)((s->addr << 1) | 1),  /* read address */
        rx[0], rx[1]
    };
    if (crc8(frame, sizeof(frame)) != rx[2]) {
        ESP_LOGW(TAG, "PEC mismatch on cmd 0x%02X (addr 0x%02X)", command, s->addr);
        return ESP_ERR_INVALID_CRC;
    }

    *value = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    /* Error flag: MSB set on temperature reads indicates a sensor fault */
    if ((command == MLX90614_RAM_TOBJ1 || command == MLX90614_RAM_TA) &&
        (*value & 0x8000)) {
        ESP_LOGW(TAG, "Sensor error flag set (raw 0x%04X)", *value);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t read_temp_c(mlx90614_t *s, uint8_t reg, float *temp_c)
{
    uint16_t raw;
    esp_err_t err = mlx90614_read_word(s, reg, &raw);
    if (err != ESP_OK) {
        return err;
    }
    *temp_c = (float)raw * 0.02f - 273.15f;
    return ESP_OK;
}

esp_err_t mlx90614_read_object_c(mlx90614_t *s, float *temp_c)
{
    return read_temp_c(s, MLX90614_RAM_TOBJ1, temp_c);
}

esp_err_t mlx90614_read_ambient_c(mlx90614_t *s, float *temp_c)
{
    return read_temp_c(s, MLX90614_RAM_TA, temp_c);
}

/* Internal: SMBus Write Word with PEC */
static esp_err_t write_word(i2c_master_dev_handle_t dev, uint8_t addr7,
                            uint8_t command, uint16_t value)
{
    uint8_t frame_for_crc[4] = {
        (uint8_t)(addr7 << 1), command,
        (uint8_t)(value & 0xFF), (uint8_t)(value >> 8)
    };
    uint8_t tx[4] = {
        command,
        (uint8_t)(value & 0xFF),
        (uint8_t)(value >> 8),
        crc8(frame_for_crc, sizeof(frame_for_crc))
    };
    return i2c_master_transmit(dev, tx, sizeof(tx), I2C_TIMEOUT_MS);
}

esp_err_t mlx90614_set_smbus_address(i2c_master_bus_handle_t bus, uint8_t new_addr)
{
    if (new_addr < 0x08 || new_addr > 0x77) {
        return ESP_ERR_INVALID_ARG;
    }

    mlx90614_t bc;
    esp_err_t err = mlx90614_init(bus, MLX90614_BROADCAST_ADDR, &bc);
    if (err != ESP_OK) return err;

    /* Erase EEPROM cell first (write 0x0000), then write new address */
    err = write_word(bc.dev, MLX90614_BROADCAST_ADDR,
                     MLX90614_EEPROM_SMBUS_ADDR, 0x0000);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(EEPROM_WRITE_DELAY_MS));
        err = write_word(bc.dev, MLX90614_BROADCAST_ADDR,
                         MLX90614_EEPROM_SMBUS_ADDR, (uint16_t)new_addr);
        vTaskDelay(pdMS_TO_TICKS(EEPROM_WRITE_DELAY_MS));
    }
    i2c_master_bus_rm_device(bc.dev);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SMBus address set to 0x%02X - power-cycle the sensor now",
                 new_addr);
    }
    return err;
}

esp_err_t mlx90614_set_emissivity(mlx90614_t *s, float emissivity)
{
    if (emissivity < 0.1f || emissivity > 1.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t e = (uint16_t)(emissivity * 65535.0f);

    /* Erase, then write */
    esp_err_t err = write_word(s->dev, s->addr, MLX90614_EEPROM_EMISSIVITY, 0x0000);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(EEPROM_WRITE_DELAY_MS));

    err = write_word(s->dev, s->addr, MLX90614_EEPROM_EMISSIVITY, e);
    vTaskDelay(pdMS_TO_TICKS(EEPROM_WRITE_DELAY_MS));

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Emissivity set to %.2f (0x%04X) - power-cycle the sensor",
                 emissivity, e);
    }
    return err;
}
