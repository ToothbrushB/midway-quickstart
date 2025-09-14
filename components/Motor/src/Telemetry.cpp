#include "Telemetry.hpp"
#include "esp_crt_bundle.h"
#include "mqtt5_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include <WifiHelper.hpp>
#include <esp_mac.h>
#include "esp_timer.h"
#include <queue>
#include <mutex>
#include <inttypes.h>
#include <sys/time.h>

static const char *TAG = "Telemetry";
esp_mqtt_client_handle_t Telemetry::client; // Static member variable definition
bool Telemetry::isConnected = false;        // Static member variable definition
bool Telemetry::isInit = false;             // Static member variable definition

std::map<SubscriptionHandle, std::pair<const char *, SubscriptionCallback>> Telemetry::subscriptions;
SubscriptionHandle Telemetry::nextHandle = 0; // Initialize static handle counter
char *Telemetry::BASE_TOPIC;
uint64_t Telemetry::lastPingTime = 0;      // Initialize last ping time
esp_timer_handle_t Telemetry::timerHandle; // Timer handle for periodic tasks
SubscriptionHandle Telemetry::ping = -1;   // Subscription handle for ping messages

// Queue-related static members
std::queue<QueuedMessage> Telemetry::messageQueue;
std::mutex Telemetry::queueMutex;
TaskHandle_t Telemetry::publishTaskHandle = nullptr;

// Periodic publishing static members
std::map<PeriodicHandle, PeriodicCallbackInfo> Telemetry::periodicCallbacks;
SemaphoreHandle_t Telemetry::periodicMutex;
PeriodicHandle Telemetry::nextPeriodicHandle = 1;
TaskHandle_t Telemetry::periodicTaskHandle = nullptr;
bool Telemetry::periodicTaskRunning = false;
static esp_mqtt5_user_property_item_t user_property_arr[] = {
    {"timestamp", "0000000"},
};

#define USE_PROPERTY_ARR_SIZE sizeof(user_property_arr) / sizeof(esp_mqtt5_user_property_item_t)

static esp_mqtt5_publish_property_config_t publish_property = {
    .payload_format_indicator = 1,
    .message_expiry_interval = 1000,
    .topic_alias = 0,
    .response_topic = "",
    .correlation_data = "",
    .correlation_data_len = 0,
};

static void print_user_property(mqtt5_user_property_handle_t user_property)
{
    if (user_property)
    {
        uint8_t count = esp_mqtt5_client_get_user_property_count(user_property);
        if (count)
        {
            esp_mqtt5_user_property_item_t *item = (esp_mqtt5_user_property_item_t *)malloc(count * sizeof(esp_mqtt5_user_property_item_t));
            if (esp_mqtt5_client_get_user_property(user_property, item, &count) == ESP_OK)
            {
                for (int i = 0; i < count; i++)
                {
                    esp_mqtt5_user_property_item_t *t = &item[i];
                    ESP_LOGI(TAG, "key is %s, value is %s", t->key, t->value);
                    free((char *)t->key);
                    free((char *)t->value);
                }
            }
            free(item);
        }
    }
}

/***************************
 * Subscription management *
 ****************************/
SubscriptionHandle Telemetry::subscribe(const char *topic, SubscriptionCallback callback)
{
    if (!isConnected)
    {
        ESP_LOGE(TAG, "Cannot subscribe - MQTT not connected");
        return -1;
    }

    // Create full topic with base
    char *fullTopic = (char *)malloc(strlen(BASE_TOPIC) + strlen(topic) + 1);
    strcpy(fullTopic, BASE_TOPIC);
    strcat(fullTopic, topic);

    // Subscribe to MQTT topic
    int msg_id = esp_mqtt_client_subscribe(client, fullTopic, 1);
    if (msg_id == -1)
    {
        ESP_LOGE(TAG, "Failed to subscribe to topic: %s", fullTopic);
        free(fullTopic);
        return -1;
    }

    // Store subscription
    SubscriptionHandle handle = nextHandle++;
    subscriptions[handle] = std::make_pair(strdup(topic), callback);

    ESP_LOGI(TAG, "Subscribed to topic: %s with handle: %d", fullTopic, handle);
    free(fullTopic);
    return handle;
}
// Add unsubscribe method
bool Telemetry::unsubscribe(SubscriptionHandle handle)
{
    auto it = subscriptions.find(handle);
    if (it == subscriptions.end())
    {
        ESP_LOGE(TAG, "Invalid subscription handle: %d", handle);
        return false;
    }

    // Unsubscribe from MQTT
    char *fullTopic = (char *)malloc(strlen(BASE_TOPIC) + strlen(it->second.first) + 1);
    strcpy(fullTopic, BASE_TOPIC);
    strcat(fullTopic, it->second.first);

    int msg_id = esp_mqtt_client_unsubscribe(client, fullTopic);

    // Remove from subscriptions map

    ESP_LOGI(TAG, "Unsubscribed from topic: %s", fullTopic);
    free(fullTopic);
    free((char *)it->second.first); // Free duplicated topic string
    subscriptions.erase(it);

    return (msg_id != -1);
}

void Telemetry::handleReceivedData(const char *topic, const char *data, int data_len)
{
    // Extract relative topic (remove BASE_TOPIC prefix)
    const char *relativeTopic = topic;
    size_t baseLen = strlen(BASE_TOPIC);
    if (strncmp(topic, BASE_TOPIC, baseLen) == 0)
    {
        relativeTopic = topic + baseLen;
    }
    // Find matching subscriptions and call callbacks
    for (const auto &sub : subscriptions)
    {
        if (strcmp(sub.second.first, relativeTopic) == 0)
        {
            // Call the callback
            sub.second.second(relativeTopic, data, data_len);
        }
    }
}

/***************************
 * Periodic publishing API *
 ****************************/
PeriodicHandle Telemetry::registerPeriodicCallback(PeriodicCallback callback, PublishFrequency frequency)
{
    if (!isInit)
    {
        ESP_LOGE(TAG, "Telemetry not initialized, cannot register periodic callback");
        return -1;
    }
    ESP_LOGI(TAG, "Trying to acquire periodicMutex");
    xSemaphoreTake(periodicMutex, portMAX_DELAY);
    ESP_LOGI(TAG, "Acquired periodicMutex");
    PeriodicHandle handle = nextPeriodicHandle++;

    PeriodicCallbackInfo info;
    info.callback = callback;
    info.frequency = frequency;
    info.lastExecuted = 0;
    info.active = true;

    periodicCallbacks[handle] = info;

    ESP_LOGI(TAG, "Registered periodic callback with handle: %d, frequency: %d us", handle, (int)frequency);

    xSemaphoreGive(periodicMutex);
    // Start periodic task if not already running
    if (!periodicTaskRunning)
    {
        startPeriodicPublishing();
    }

    return handle;
}

bool Telemetry::unregisterPeriodicCallback(PeriodicHandle handle)
{
    xSemaphoreTake(periodicMutex, portMAX_DELAY);
    auto it = periodicCallbacks.find(handle);
    if (it == periodicCallbacks.end())
    {
        ESP_LOGE(TAG, "Invalid periodic handle: %d", handle);
        return false;
    }

    periodicCallbacks.erase(it);
    ESP_LOGI(TAG, "Unregistered periodic callback with handle: %d", handle);
    xSemaphoreGive(periodicMutex);

    return true;
}

void Telemetry::startPeriodicPublishing()
{
    if (!periodicTaskRunning)
    {
        periodicTaskRunning = true;
        xTaskCreate(periodicTask, "TelemetryPeriodic", 8192, NULL, 6, &periodicTaskHandle);
        ESP_LOGI(TAG, "Periodic publishing task started");
    }
}

void Telemetry::stopPeriodicPublishing()
{
    if (periodicTaskRunning)
    {
        periodicTaskRunning = false;
        if (periodicTaskHandle != nullptr)
        {
            vTaskDelete(periodicTaskHandle);
            periodicTaskHandle = nullptr;
        }
        ESP_LOGI(TAG, "Periodic publishing task stopped");
    }
}

void Telemetry::periodicTask(void *parameter)
{
    ESP_LOGI(TAG, "Periodic publish task started");

    while (periodicTaskRunning)
    {
        static int log_counter = 0;
        // if (++log_counter % 100 == 0) { // Every 100 iterations
        //     ESP_LOGI(TAG, "Free heap: %lu bytes, Queue size: %u",
        //              esp_get_free_heap_size(), messageQueue.size());
        // }
        // Process all periodic callbacks
        xSemaphoreTake(periodicMutex, portMAX_DELAY);

        for (auto &pair : periodicCallbacks)
        {
            PeriodicCallbackInfo &info = pair.second;

            if (!isConnected)
                continue;

            if (!info.active)
            {
                continue;
            }

            // Check if it's time to execute this callback
            uint64_t timeSinceLastExecution = esp_timer_get_time() - info.lastExecuted;

            if (timeSinceLastExecution >= (uint64_t)info.frequency)
            {
                // Execute the callback
                info.callback();
                info.lastExecuted = esp_timer_get_time();
            }
        }
        xSemaphoreGive(periodicMutex);

        vTaskDelay(10 / portTICK_PERIOD_MS); // Yield to other tasks
    }

    ESP_LOGI(TAG, "Periodic publish task ended");
    vTaskDelete(NULL);
}

/***************************
 * Publish management *
 ****************************/
struct timeval tv_now;
void Telemetry::queueMessage(const char *topic, const char *payload, int qos)
{
    if (!isConnected)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(queueMutex);
    size_t free_heap = esp_get_free_heap_size();
    if (messageQueue.size() >= 100)
    { // Limit queue size
        ESP_LOGW(TAG, "Message queue full (%zu messages), dropping oldest message. %zu bytes", messageQueue.size(), free_heap);
        QueuedMessage dropped = messageQueue.front();
        free(dropped.topic);
        free((char *)dropped.payload);
        messageQueue.pop(); // Remove oldest message
    }
    if (free_heap < 1024)
    { // Less than 1KB free

        // Clear the entire queue to free memory
        while (!messageQueue.empty())
        {
            QueuedMessage dropped = messageQueue.front();
            free(dropped.topic);
            free((char *)dropped.payload);
            messageQueue.pop();
        }
        ESP_LOGE(TAG, "Low memory (%zu bytes), clearing queue", free_heap);
        return;
    }
    QueuedMessage message;
    message.topic = (char *)malloc(strlen(BASE_TOPIC) + strlen(topic) + 1);
    strcpy(message.topic, BASE_TOPIC);
    strcat(message.topic, topic);
    message.payload = strdup(payload);
    message.qos = qos;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    message.timestamp = time_us; // Current time in microseconds

    messageQueue.push(message);

    // Notify the publish task that there's a new message
    if (publishTaskHandle != nullptr)
    {
        xTaskNotifyGive(publishTaskHandle);
    }
}

bool Telemetry::publishMessageWithTimestamp(const QueuedMessage &message)
{
    if (!isConnected)
    {
        return false;
    }

    // Create user properties for MQTT v5
    user_property_arr[0].key = "timestamp";

    // Convert timestamp to string
    char timestamp_str[32];
    snprintf(timestamp_str, sizeof(timestamp_str), "%" PRIu64, message.timestamp);
    user_property_arr[0].value = timestamp_str;

    // Publish with user properties
    esp_mqtt5_client_set_user_property(&publish_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_publish_property(client, &publish_property);
    int msg_id = esp_mqtt_client_publish(client, message.topic, message.payload, 0, message.qos, 0);
    esp_mqtt5_client_delete_user_property(publish_property.user_property); // Clean up user properties
    publish_property.user_property = NULL;

    if (msg_id == -1)
    {
        ESP_LOGE(TAG, "Failed to publish message to topic: %s", message.topic);
        return false;
    }

    return true;
}

void Telemetry::publishTask(void *parameter)
{
    ESP_LOGI(TAG, "Publish task started");

    while (true)
    {
        // Wait for notification that there's a message to publish
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Process all queued messages
        while (true)
        {
            QueuedMessage message;
            bool hasMessage = false;

            // Get message from queue
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                if (!messageQueue.empty())
                {
                    message = messageQueue.front();
                    messageQueue.pop();
                    hasMessage = true;
                }
            }

            if (!hasMessage)
            {
                break; // No more messages
            }

            // Try to publish the message
            publishMessageWithTimestamp(message);

            free(message.topic);
            free((char *)message.payload);

            // Small delay between messages to avoid flooding
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void Telemetry::publishData(const char *topic, const char *payload, int qos)
{
    // Queue data for publishing with timestamp
    // queueMessage(topic, payload, qos);
}

void Telemetry::publishData(const char *topic, double payload, int qos)
{
    // Convert double to string and queue for publishing
    char payload_str[32];
    snprintf(payload_str, sizeof(payload_str), "%.6f", payload);
    // queueMessage(topic, payload_str, qos);
}

void Telemetry::publishData(const char *topic, int payload, int qos)
{
    // Convert int to string and queue for publishing
    char payload_str[32];
    snprintf(payload_str, sizeof(payload_str), "%d", payload);
    // queueMessage(topic, payload_str, qos);
}

void Telemetry::publishData(const char *topic, bool payload, int qos)
{
    // Convert bool to string and queue for publishing
    const char *payload_str = payload ? "true" : "false";
    // queueMessage(topic, payload_str, qos);
}
/******************************
 * MQTT event handling & init *
 ******************************
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

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        isConnected = true;

        // Publish "Device started" message on connect
        char *topic = (char *)malloc(strlen(BASE_TOPIC) + strlen("status") + 1);
        strcpy(topic, BASE_TOPIC);
        strcat(topic, "status");
        esp_mqtt_client_publish(client, topic, "Connected", 0, 1, 0);
        free(topic);

        // Notify publish task to process any queued messages
        if (publishTaskHandle != nullptr)
        {
            xTaskNotifyGive(publishTaskHandle);
        }

        if (ping != -1)
        {
            ESP_LOGI(TAG, "Re-subscribing to ping topic");
            unsubscribe(ping); // Unsubscribe first if already subscribed
        }
        ping = subscribe("ping", [](const char *topic, const char *data, int data_len)
                         {
            if (strncmp(data, "ping", data_len) == 0) {
                publishData("ping", "pong", 1);
                lastPingTime = esp_timer_get_time();
            } else { /* ignore all pongs */ } });

        // esp_timer_create_args_t timer_args = {
        //     .callback = [](void* arg) {
        //         // Check if we need to send a ping
        //         if (esp_timer_get_time() - lastPingTime > 10e6) { // 10 seconds
        //             ESP_LOGW(TAG, "No ping received in the last 10 seconds..");
        //         }
        //     },
        //     .arg = NULL,
        //     .dispatch_method = ESP_TIMER_TASK,
        //     .name = "TelemetryPingTimer",
        //     .skip_unhandled_events = false,
        // };
        // esp_timer_create(&timer_args, &timerHandle);
        // esp_timer_start_periodic(timerHandle, 1000000); // 1 second interval
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        if (ping != -1)
        {
            ESP_LOGI(TAG, "Re-subscribing to ping topic");
            unsubscribe(ping); // Unsubscribe first if already subscribed
        }
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
        if (event->protocol_ver == MQTT_PROTOCOL_V_5)
        {
            print_user_property(event->property->user_property);
        }
        handleReceivedData(event->topic, event->data, event->data_len);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)", event->error_handle->esp_transport_sock_errno, strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
// Must be inited after Wi-Fi is connected.
void Telemetry::init()
{
    // Initialization code

    uint8_t mac[6];
    BASE_TOPIC = new char[32];
    esp_efuse_mac_get_default(mac);
    snprintf(BASE_TOPIC, 32, "robots/%02x%02x%02x%02x%02x%02x/", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    char *lastWill = new char[50];
    snprintf(lastWill, 50, "robots/%02x%02x%02x%02x%02x%02x/status", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {.uri = CONFIG_BROKER_URI}
            // .verification = {.crt_bundle_attach = esp_crt_bundle_attach}
        },
        .credentials = {.username = "esp", .authentication = {.password = "esp32"}},
        .session = {
            .last_will = {.topic = lastWill, .msg = "Disconnected!", .msg_len = 0, .qos = 1, .retain = 0},
            .protocol_ver = MQTT_PROTOCOL_V_5, // Use MQTT v5 protocol
        }};

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt5_event_handler, NULL);
    esp_mqtt_client_start(client);

    // Start the publish task
    periodicMutex = xSemaphoreCreateMutex();
    xTaskCreate(publishTask, "TelemetryPublish", 16384, NULL, 5, &publishTaskHandle);
    ESP_LOGI(TAG, "Telemetry publish task created");
    isInit = true; // Mark Telemetry as initialized
}

void Telemetry::waitForConnection()
{
    while (!isConnected)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Waiting for MQTT connection...");
    }
}