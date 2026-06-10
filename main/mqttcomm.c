#include "mqttcomm.h"
#include "mqtt_client.h"
#include "esp_log.h"

#define BROKER_URI         "mqtt://broker:1883"
#define CLIENT_ID          "tire-temp"
#define STATUS_TOPIC       "fiesta/device/tire-temp/status"
#define STATUS_ONLINE      "{\"status\":\"online\"}"
#define STATUS_OFFLINE     "{\"status\":\"offline\"}"

static const char* TAG = "mqttcomm";
static esp_mqtt_client_handle_t client = NULL;

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            esp_mqtt_client_publish(client, STATUS_TOPIC, STATUS_ONLINE, 0, 1, 1);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGE(TAG, "MQTT disconnected, retrying in ~5s...");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;
        default:
            break;
    }
}

void mqttcomm_start(void) {
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URI,
        .credentials.client_id = CLIENT_ID,
        .session.last_will = {
            .topic = STATUS_TOPIC,
            .msg   = STATUS_OFFLINE,
            .qos   = 1,
            .retain = 1,
        },
        .network.reconnect_timeout_ms = 5000,
    };

    ESP_LOGI(TAG, "Connecting to MQTT broker at %s", BROKER_URI);
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

int mqttcomm_publish(const char* topic, const char* data, int len) {
    if (client == NULL) {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return -1;
    }
    return esp_mqtt_client_publish(client, topic, data, len, 1, 0);
}
