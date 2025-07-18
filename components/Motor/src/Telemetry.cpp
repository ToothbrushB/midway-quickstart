#include "Telemetry.hpp"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include <WifiHelper.hpp>
#include <string>
#include <esp_mac.h>
#include "esp_timer.h"
#include <queue>
#include <mutex>
#include <inttypes.h>

static const char* TAG = "Telemetry";
esp_mqtt_client_handle_t Telemetry::client; // Static member variable definition
bool Telemetry::isConnected = false; // Static member variable definition

std::map<SubscriptionHandle, std::pair<std::string, SubscriptionCallback>> Telemetry::subscriptions;
SubscriptionHandle Telemetry::nextHandle = 0; // Initialize static handle counter
std::string Telemetry::BASE_TOPIC;
uint64_t Telemetry::lastPingTime = 0; // Initialize last ping time
esp_timer_handle_t Telemetry::timerHandle; // Timer handle for periodic tasks
SubscriptionHandle Telemetry::ping = -1; // Subscription handle for ping messages

// Queue-related static members
std::queue<QueuedMessage> Telemetry::messageQueue;
std::mutex Telemetry::queueMutex;
TaskHandle_t Telemetry::publishTaskHandle = nullptr;

SubscriptionHandle Telemetry::subscribe(const char* topic, SubscriptionCallback callback) {
    if (!isConnected) {
        ESP_LOGE(TAG, "Cannot subscribe - MQTT not connected");
        return -1;
    }
    
    // Create full topic with base
    std::string fullTopic = BASE_TOPIC + topic;
    
    // Subscribe to MQTT topic
    int msg_id = esp_mqtt_client_subscribe(client, fullTopic.c_str(), 1);
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Failed to subscribe to topic: %s", fullTopic.c_str());
        return -1;
    }
    
    // Store subscription
    SubscriptionHandle handle = nextHandle++;
    subscriptions[handle] = std::make_pair(std::string(topic), callback);
    
    ESP_LOGI(TAG, "Subscribed to topic: %s with handle: %d", fullTopic.c_str(), handle);
    return handle;
}
// Add unsubscribe method
bool Telemetry::unsubscribe(SubscriptionHandle handle) {
    auto it = subscriptions.find(handle);
    if (it == subscriptions.end()) {
        ESP_LOGE(TAG, "Invalid subscription handle: %d", handle);
        return false;
    }
    
    // Unsubscribe from MQTT
    std::string fullTopic = BASE_TOPIC + it->second.first;
    int msg_id = esp_mqtt_client_unsubscribe(client, fullTopic.c_str());
    
    // Remove from subscriptions map
    subscriptions.erase(it);
    
    ESP_LOGI(TAG, "Unsubscribed from topic: %s", fullTopic.c_str());
    return (msg_id != -1);
}

void Telemetry::handleReceivedData(const char* topic, const char* data, int data_len) {
    // Extract relative topic (remove BASE_TOPIC prefix)
    std::string fullTopic(topic);
    std::string relativeTopic;
    
    if (fullTopic.find(BASE_TOPIC) == 0) {
        relativeTopic = fullTopic.substr(BASE_TOPIC.length());
    } else {
        relativeTopic = fullTopic;
    }
    
    // Find matching subscriptions and call callbacks
    for (const auto& sub : subscriptions) {
        if (sub.second.first == relativeTopic) {
            // Call the callback
            sub.second.second(relativeTopic.c_str(), data, data_len);
        }
    }
}

void Telemetry::queueMessage(const char* topic, const char* payload, int qos) {
    std::lock_guard<std::mutex> lock(queueMutex);
    
    QueuedMessage message;
    message.topic = BASE_TOPIC + topic;
    message.payload = payload;
    message.qos = qos;
    message.timestamp = esp_timer_get_time(); // Current time in microseconds
    
    messageQueue.push(message);
    
    // Notify the publish task that there's a new message
    if (publishTaskHandle != nullptr) {
        xTaskNotifyGive(publishTaskHandle);
    }
}

bool Telemetry::publishMessageWithTimestamp(const QueuedMessage& message) {
    if (!isConnected) {
        return false;
    }
    
    // Create user properties for MQTT v5
    esp_mqtt5_user_property_item_t user_property = {0};
    user_property.key = "timestamp";
    
    // Convert timestamp to string
    char timestamp_str[32];
    snprintf(timestamp_str, sizeof(timestamp_str), "%" PRIu64, message.timestamp);
    user_property.value = timestamp_str;
    
    // Publish with user properties
    esp_mqtt5_client_set_user_property(&user_property);
    int msg_id = esp_mqtt_client_publish(
        client,
        message.topic.c_str(),
        message.payload.c_str(),
        message.payload.length(),
        message.qos,
        0, // retain
    );
    esp_mqtt5_client_delete_user_property(&user_property)
    
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Failed to publish message to topic: %s", message.topic.c_str());
        return false;
    }
    
    return true;
}

void Telemetry::publishTask(void* parameter) {
    ESP_LOGI(TAG, "Publish task started");
    
    while (true) {
        // Wait for notification that there's a message to publish
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Process all queued messages
        while (true) {
            QueuedMessage message;
            bool hasMessage = false;
            
            // Get message from queue
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                if (!messageQueue.empty()) {
                    message = messageQueue.front();
                    messageQueue.pop();
                    hasMessage = true;
                }
            }
            
            if (!hasMessage) {
                break; // No more messages
            }
            
            // Try to publish the message
            if (!publishMessageWithTimestamp(message)) {
                // If publish failed, put it back in the queue
                std::lock_guard<std::mutex> lock(queueMutex);
                messageQueue.push(message);
                ESP_LOGW(TAG, "Failed to publish message, re-queued");
                break; // Stop processing for now
            }
            
            // Small delay between messages to avoid flooding
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
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
void Telemetry::mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED: {
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        isConnected = true;
        esp_mqtt_client_publish(client, (BASE_TOPIC + "status").c_str(), "Device started", 0, 1, 0);
        
        // Notify publish task to process any queued messages
        if (publishTaskHandle != nullptr) {
            xTaskNotifyGive(publishTaskHandle);
        }
        
        if (ping != -1) {
            ESP_LOGI(TAG, "Re-subscribing to ping topic");
            unsubscribe(ping); // Unsubscribe first if already subscribed
        }
        ping = subscribe("ping", [](const char* topic, const char* data, int data_len) {
            if (strncmp(data, "ping", data_len) == 0) {
                publishData("ping", "pong", 1);
                lastPingTime = esp_timer_get_time();
            } else { /* ignore all pongs */ }
        });
        
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                // Check if we need to send a ping
                if (esp_timer_get_time() - lastPingTime > 10e6) { // 10 seconds
                    ESP_LOGE(TAG, "No ping received in the last 10 seconds.."); 
                }
            },
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "TelemetryPingTimer",
            .skip_unhandled_events = false,
        };
        esp_timer_create(&timer_args, &timerHandle);
        esp_timer_start_periodic(timerHandle, 1000000); // 1 second interval
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        isConnected = false;
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
        handleReceivedData(event->topic, event->data, event->data_len);
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
                .msg = "Disconnected!",
                .msg_len = 0,
                .qos = 1,
                .retain = 0
            },
            .protocol_ver = MQTT_PROTOCOL_V_5, // Use MQTT v5 protocol
        }
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    uint8_t mac[6];
    char macString[13];
    esp_efuse_mac_get_default(mac);
    snprintf(macString, sizeof(macString), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Telemetry::BASE_TOPIC = std::string(macString) + "/"; // Use the MAC address as the base topic
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt5_event_handler, NULL);
    esp_mqtt_client_start(client);
    
    // Start the publish task
    xTaskCreate(publishTask, "TelemetryPublish", 4096, NULL, 5, &publishTaskHandle);
    ESP_LOGI(TAG, "Telemetry publish task created");
}



void Telemetry::waitForConnection() {
    while (!isConnected) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Waiting for MQTT connection...");
    }
}


void Telemetry::publishData(const char* topic, const char* payload, int qos) {
    // Queue data for publishing with timestamp
    queueMessage(topic, payload, qos);
}


void Telemetry::publishData(const char* topic, double payload, int qos) {
    // Convert double to string and queue for publishing
    char payload_str[32];
    snprintf(payload_str, sizeof(payload_str), "%.2f", payload);
    queueMessage(topic, payload_str, qos);
}

void Telemetry::publishData(const char* topic, int payload, int qos) {
    // Convert int to string and queue for publishing
    char payload_str[32];
    snprintf(payload_str, sizeof(payload_str), "%d", payload);
    queueMessage(topic, payload_str, qos);
}

void Telemetry::publishData(const char* topic, bool payload, int qos) {
    // Convert bool to string and queue for publishing
    const char* payload_str = payload ? "true" : "false";
    queueMessage(topic, payload_str, qos);
}

void Telemetry::publishData(const char* topic, std::string payload, int qos) {
    // Queue string data for publishing
    queueMessage(topic, payload.c_str(), qos);
}

