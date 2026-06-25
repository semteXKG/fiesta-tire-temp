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
#define EMISSIVITY       0.95f
#define REFLECTED_TEMP_C 25.0f

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

    static mlx90640_t sensor;
    static float matrix[MLX90640_PIXELS];
    static int frame_count = 0;

    while (1) {
        esp_err_t err = mlx90640_init(bus, SENSOR_ADDR, &sensor);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "MLX90640 init failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
            continue;
        }

        ESP_LOGI(TAG, "MLX90640 attached @ 0x%02X", SENSOR_ADDR);

        while (1) {
            err = ESP_OK;
            float ta = 0.0f;
            for (int retry = 0; retry < 3; retry++) {
                err = mlx90640_read_frame(&sensor, matrix, EMISSIVITY, REFLECTED_TEMP_C, &ta);
                if (err == ESP_OK) {
                    break;
                }
                ESP_LOGW(TAG, "frame read failed (attempt %d): %s", retry + 1, esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "frame read failed after retries: %s", esp_err_to_name(err));
                break;
            }

            float min = matrix[0];
            float max = matrix[0];
            float sum = 0.0f;
            for (int i = 0; i < MLX90640_PIXELS; i++) {
                if (matrix[i] < min) min = matrix[i];
                if (matrix[i] > max) max = matrix[i];
                sum += matrix[i];
            }
            ESP_LOGI(TAG, "frame %d  Ta=%.1f  min=%.1f  max=%.1f  avg=%.1f",
                     frame_count++, ta, min, max, sum / MLX90640_PIXELS);

            for (int row = 0; row < MLX90640_ROWS; row++) {
                char line[256];
                int pos = snprintf(line, sizeof(line), "row %02d:", row);
                for (int col = 0; col < MLX90640_COLS && pos < (int)sizeof(line) - 8; col++) {
                    pos += snprintf(line + pos, sizeof(line) - pos, " %.1f",
                                    matrix[row * MLX90640_COLS + col]);
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
