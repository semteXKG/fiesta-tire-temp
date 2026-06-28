#include <stdio.h>
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "wlan.h"
#include "mqttcomm.h"
#include "tire_temp.h"

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wlan_start();
    mqttcomm_start();
    tire_temp_start();
}
