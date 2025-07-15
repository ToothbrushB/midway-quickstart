#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#pragma once

#include <functional>
#include <string>
class Telemetry 
{
public:
    using MessageCallback = std::function<void(const char* topic, const char* payload)>;


    static void init();
    static void waitForConnection();
    static void deinit();
    static void publishData(const char* topic, const char* payload);
    static void publishData(const char* topic,double payload);
    static void publishData(const char* topic,int payload);
    static void publishData(const char* topic,bool payload);

    static void publishData(const char *topic, std::string payload);

    static void registerCallback(const char* topic, MessageCallback callback);

private:
    static esp_mqtt_client_handle_t client;
    static bool isConnected;
    static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
    static std::string BASE_TOPIC; // Base topic for MQTT messages
};
