/**
 * Telemetry Periodic Publishing API Usage Example
 * 
 * This example demonstrates how to register callbacks that run at different frequencies
 * for automatic periodic telemetry publishing.
 */

#include "Telemetry.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "TelemetryPeriodicExample";

class SensorManager {
private:
    static float temperature;
    static float humidity;
    static int counter;
    static PeriodicHandle tempHandle;
    static PeriodicHandle humidityHandle;
    static PeriodicHandle counterHandle;

public:
    static void init() {
        ESP_LOGI(TAG, "Initializing sensor manager with periodic telemetry");
        
        // Register high frequency sensor data (100Hz - every 10ms)
        tempHandle = Telemetry::registerPeriodicCallback([]() {
            // Simulate reading temperature sensor
            temperature = 20.0 + (esp_timer_get_time() % 1000000) / 100000.0f;
            Telemetry::publishData("sensors/temperature", temperature);
        }, PublishFrequency::HZ_100);
        
        // Register medium frequency sensor data (50Hz - every 20ms)  
        humidityHandle = Telemetry::registerPeriodicCallback([]() {
            // Simulate reading humidity sensor
            humidity = 45.0 + (esp_timer_get_time() % 2000000) / 200000.0f;
            Telemetry::publishData("sensors/humidity", humidity);
        }, PublishFrequency::HZ_50);
        
        // Register low frequency status data (10Hz - every 100ms)
        counterHandle = Telemetry::registerPeriodicCallback([]() {
            // Publish system status/counters
            counter++;
            Telemetry::publishData("system/heartbeat", counter);
            Telemetry::publishData("system/uptime", (int)(esp_timer_get_time() / 1000000));
            Telemetry::publishData("system/free_heap", (int)esp_get_free_heap_size());
        }, PublishFrequency::HZ_10);
        
        ESP_LOGI(TAG, "Registered periodic callbacks - temp: %d, humidity: %d, counter: %d", 
                 tempHandle, humidityHandle, counterHandle);
    }
    
    static void cleanup() {
        ESP_LOGI(TAG, "Cleaning up sensor manager");
        
        // Unregister all periodic callbacks
        if (tempHandle != -1) {
            Telemetry::unregisterPeriodicCallback(tempHandle);
        }
        if (humidityHandle != -1) {
            Telemetry::unregisterPeriodicCallback(humidityHandle);
        }
        if (counterHandle != -1) {
            Telemetry::unregisterPeriodicCallback(counterHandle);
        }
    }
    
    // Example of conditional publishing
    static void registerConditionalTelemetry() {
        Telemetry::registerPeriodicCallback([]() {
            // Only publish if temperature is above threshold
            if (temperature > 25.0f) {
                Telemetry::publishData("alerts/high_temp", temperature, 1); // High QoS for alerts
            }
            
            // Publish diagnostic data with structured format
            char diagnostics[128];
            snprintf(diagnostics, sizeof(diagnostics), 
                    "{\"temp\":%.2f,\"humidity\":%.2f,\"uptime\":%llu}", 
                    temperature, humidity, esp_timer_get_time() / 1000000);
            Telemetry::publishData("diagnostics/sensors", diagnostics);
            
        }, PublishFrequency::HZ_10);
    }
};

// Static member definitions
float SensorManager::temperature = 0.0f;
float SensorManager::humidity = 0.0f;
int SensorManager::counter = 0;
PeriodicHandle SensorManager::tempHandle = -1;
PeriodicHandle SensorManager::humidityHandle = -1;
PeriodicHandle SensorManager::counterHandle = -1;

/**
 * Usage in main application:
 * 
 * void app_main() {
 *     // Initialize WiFi and Telemetry
 *     WifiHelper::init();
 *     WifiHelper::waitForConnection();
 *     Telemetry::init();
 *     Telemetry::waitForConnection();
 *     
 *     // Initialize periodic sensor publishing
 *     SensorManager::init();
 *     
 *     // Your main application loop
 *     while (true) {
 *         // Do other work...
 *         vTaskDelay(1000 / portTICK_PERIOD_MS);
 *     }
 * }
 */

/**
 * Benefits of this approach:
 * 
 * 1. **Automatic Publishing**: No need to manually call publish functions in loops
 * 2. **Precise Timing**: High-resolution timing for consistent intervals
 * 3. **Different Frequencies**: Mix high-frequency sensor data with low-frequency status
 * 4. **Timestamped**: All messages automatically include creation timestamps
 * 5. **Queue Resilient**: Messages are queued if MQTT connection is down
 * 6. **Easy Management**: Simple register/unregister API with handles
 * 7. **Exception Safe**: Callbacks are wrapped in try-catch blocks
 * 8. **Non-blocking**: Runs in separate task, doesn't block main application
 */
