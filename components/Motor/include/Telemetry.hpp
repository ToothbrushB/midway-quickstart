#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include <functional>
#include <string>
#include <map>
#include <queue>
#include <mutex>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#pragma once


typedef std::function<void(const char* topic, const char* data, int data_len)> SubscriptionCallback;
typedef int SubscriptionHandle;

// Structure for queued messages
struct QueuedMessage {
    std::string topic;
    std::string payload;
    int qos;
    uint64_t timestamp; // When the message was queued
};

class Telemetry 
{
public:
    static void init();
    static void waitForConnection();
    static void publishData(const char* topic, const char* payload,int qos=0);
    static void publishData(const char* topic,double payload,int qos=0);
    static void publishData(const char* topic,int payload,int qos=0);
    static void publishData(const char* topic,bool payload,int qos=0);
    static void publishData(const char *topic, std::string payload, int qos=0);

    static SubscriptionHandle subscribe(const char* topic, SubscriptionCallback callback);
    static bool unsubscribe(SubscriptionHandle handle);
private:
    static esp_mqtt_client_handle_t client;
    static bool isConnected;
    static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
    static std::string BASE_TOPIC; // Base topic for MQTT messages
    static std::map<SubscriptionHandle, std::pair<std::string, SubscriptionCallback>> subscriptions;
    static SubscriptionHandle nextHandle;
    static void handleReceivedData(const char* topic, const char* data, int data_len);
    static uint64_t lastPingTime;
    static esp_timer_handle_t timerHandle; // Timer handle for periodic tasks
    static SubscriptionHandle ping;
    
    // Queue-related members
    static std::queue<QueuedMessage> messageQueue;
    static std::mutex queueMutex;
    static TaskHandle_t publishTaskHandle;
    static void publishTask(void* parameter);
    static void queueMessage(const char* topic, const char* payload, int qos);
    static bool publishMessageWithTimestamp(const QueuedMessage& message);
};
