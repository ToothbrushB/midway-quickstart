#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#pragma once

#include <functional>

class Telemetry 
{
public:
    using MessageCallback = std::function<void(const char* topic, const char* payload)>;

    static void init();
    static void deinit();
    static void publishData(const char* topic, const char* payload);
    static void registerCallback(const char* topic, MessageCallback callback);

private:
    static esp_mqtt_client_handle_t client;
    static bool isInitialized;
};
