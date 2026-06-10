/*
 * MLX90614 SMBus driver for ESP-IDF (new i2c_master API, IDF >= 5.2)
 * Tested target: MLX90614ESF-BCC (3V, 35 deg FOV, gradient compensated)
 */
#ifndef MLX90614_H
#define MLX90614_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MLX90614_DEFAULT_ADDR   0x5A
#define MLX90614_BROADCAST_ADDR 0x00   /* answered by any MLX90614 on the bus */

/* RAM registers */
#define MLX90614_RAM_TA      0x06   /* ambient (die) temperature */
#define MLX90614_RAM_TOBJ1   0x07   /* object temperature, zone 1 */

/* EEPROM registers (command = 0x20 | eeprom_addr) */
#define MLX90614_EEPROM_SMBUS_ADDR 0x2E
#define MLX90614_EEPROM_EMISSIVITY 0x24

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t addr;
} mlx90614_t;

/* Attach a sensor at `addr` to an existing I2C master bus. */
esp_err_t mlx90614_init(i2c_master_bus_handle_t bus, uint8_t addr, mlx90614_t *out);

/* Read object / ambient temperature in degrees Celsius. PEC-checked. */
esp_err_t mlx90614_read_object_c(mlx90614_t *s, float *temp_c);
esp_err_t mlx90614_read_ambient_c(mlx90614_t *s, float *temp_c);

/* Read raw 16-bit register (RAM or EEPROM command byte). */
esp_err_t mlx90614_read_word(mlx90614_t *s, uint8_t command, uint16_t *value);

/*
 * Reprogram the SMBus address stored in EEPROM.
 * IMPORTANT: only ONE sensor may be on the bus while doing this.
 * Uses the broadcast address 0x00, so it works regardless of the
 * sensor's current address. Power-cycle the sensor afterwards.
 * new_addr: 7-bit address, e.g. 0x5B, 0x5C, 0x5D
 */
esp_err_t mlx90614_set_smbus_address(i2c_master_bus_handle_t bus, uint8_t new_addr);

/*
 * Set emissivity (0.10 .. 1.00). Rubber tire tread: ~0.95 (default 1.00
 * is usually close enough, but 0.95 is more accurate for hot rubber).
 * Power-cycle afterwards.
 */
esp_err_t mlx90614_set_emissivity(mlx90614_t *s, float emissivity);

#ifdef __cplusplus
}
#endif

#endif /* MLX90614_H */
