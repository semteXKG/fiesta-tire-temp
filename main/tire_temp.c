#include "tire_temp.h"
#include "mlx90614.h"
#include "mqttcomm.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define I2C_SDA_GPIO     21
#define I2C_SCL_GPIO     22
#define POLL_PERIOD_MS   1000
#define SENSOR_ADDR      MLX90614_DEFAULT_ADDR

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

    mlx90614_t sensor;
    ESP_ERROR_CHECK(mlx90614_init(bus, SENSOR_ADDR, &sensor));
    ESP_LOGI(TAG, "MLX90614 attached @ 0x%02X", SENSOR_ADDR);

    char payload[128];
    while (1) {
        float obj, amb;
        esp_err_t e1 = mlx90614_read_object_c(&sensor, &obj);
        vTaskDelay(pdMS_TO_TICKS(2));   /* >= 1.44 ms gap required */
        esp_err_t e2 = mlx90614_read_ambient_c(&sensor, &amb);

        if (e1 == ESP_OK && e2 == ESP_OK) {
            int len = snprintf(payload, sizeof(payload),
                               "{\"obj\":%.2f,\"amb\":%.2f}", obj, amb);
            int rc = mqttcomm_publish("fiesta/tire_temp", payload, len);
            if (rc < 0) {
                ESP_LOGW(TAG, "MQTT publish failed");
            } else {
                ESP_LOGI(TAG, "published: obj=%.2f C amb=%.2f C", obj, amb);
            }
        } else {
            ESP_LOGW(TAG, "sensor read failed (obj=%s, amb=%s)",
                     esp_err_to_name(e1), esp_err_to_name(e2));
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}

void tire_temp_start(void)
{
    xTaskCreate(tire_temp_task, "tire_temp", 4096, NULL, 5, NULL);
}
