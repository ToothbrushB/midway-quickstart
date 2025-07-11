#include "Telemetry.hpp"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include <WifiHelper.hpp>
#include <string>

static const char* TAG = "Telemetry";
esp_mqtt_client_handle_t Telemetry::client; // Static member variable definition
bool Telemetry::isInitialized = false; // Static member variable definition

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static const std::string BASE_TOPIC = std::string(WifiHelper::get_mac_string()) + "/"; // Use the MAC address as the base topic
/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    ESP_LOGD(TAG, "free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        // ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
// Must be inited after Wi-Fi is connected. 
void Telemetry::init() {
    esp_log_level_set("mqtt5_client", ESP_LOG_WARN); // Set MQTT client log level to WARNING to reduce verbose output
    // Initialization code
        const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {.uri = CONFIG_BROKER_URI},
            .verification = {.crt_bundle_attach = esp_crt_bundle_attach}
        },
        .credentials = {
            .username = "esp",
            .authentication = {
                .password = "esp32"
            }
        },
        .session = {
            .last_will = {
                .topic = "last_will",
                .msg = "Device disconnected unexpectedly",
                .msg_len = 0,
                .qos = 1,
                .retain = 0
            },
            .protocol_ver = MQTT_PROTOCOL_V_5, // Use MQTT v5 protocol
        }
    };

    client = esp_mqtt_client_init(&mqtt_cfg);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt5_event_handler, NULL);

    esp_mqtt_client_start(client);
    isInitialized = true;
    esp_mqtt_client_publish(client, (BASE_TOPIC + "status").c_str(), "Device started", 0, 1, 0);
}

void Telemetry::deinit() {
    // Deinitialization code
}

void Telemetry::publishData(const char* topic, const char* payload) {
    // Publish data to MQTT topic
    if (isInitialized)
    esp_mqtt_client_publish(client, (BASE_TOPIC + topic).c_str(), payload, 0, 1, 0);
}
