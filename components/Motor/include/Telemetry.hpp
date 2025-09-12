#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include <functional>
#include <map>
#include <queue>
#include <mutex>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>
#pragma once


typedef std::function<void(const char* topic, const char* data, int data_len)> SubscriptionCallback;
typedef std::function<void()> PeriodicCallback;
typedef int SubscriptionHandle;
typedef int PeriodicHandle;

// Enum for periodic frequencies
enum class PublishFrequency {
    HZ_1 = 1000000,   // 1Hz in microseconds
    HZ_10 = 100000,   // 100ms in microseconds
    HZ_50 = 20000,    // 20ms in microseconds  
    HZ_100 = 10000    // 10ms in microseconds
};

// Structure for queued messages
struct QueuedMessage {
    char* topic; // edited to add in the base topic
    const char* payload;
    int qos;
    uint64_t timestamp; // When the message was queued
};

// Structure for periodic callbacks
struct PeriodicCallbackInfo {
    PeriodicCallback callback;
    PublishFrequency frequency;
    uint64_t lastExecuted;
    bool active;
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
    
    // Periodic publishing API
    static PeriodicHandle registerPeriodicCallback(PeriodicCallback callback, PublishFrequency frequency);
    static bool unregisterPeriodicCallback(PeriodicHandle handle);
    static void startPeriodicPublishing();
    static void stopPeriodicPublishing();
private:
    static esp_mqtt_client_handle_t client;
    static bool isConnected;
    static bool isInit;

    static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
    static char* BASE_TOPIC; // Base topic for MQTT messages
    static std::map<SubscriptionHandle, std::pair<const char*, SubscriptionCallback>> subscriptions;
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
    
    // Periodic publishing members
    static std::map<PeriodicHandle, PeriodicCallbackInfo> periodicCallbacks;
    static SemaphoreHandle_t periodicMutex;
    static PeriodicHandle nextPeriodicHandle;
    static TaskHandle_t periodicTaskHandle;
    static bool periodicTaskRunning;
    static void periodicTask(void* parameter);
};
