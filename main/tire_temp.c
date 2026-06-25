#include "tire_temp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "tire_temp";

static void tire_temp_task(void *pv)
{
    while (1) {
        ESP_LOGI(TAG, "MLX90640 driver not yet integrated");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tire_temp_start(void)
{
    xTaskCreate(tire_temp_task, "tire_temp", 4096, NULL, 5, NULL);
}
