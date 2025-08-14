/**
 * @file VL53L0X_example.cpp
 * @brief Example usage of the VL53L0X C++ class
 */

#include "VL53L0X.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "VL53L0X_EXAMPLE";

void vl53l0x_example_single_sensor() {
    ESP_LOGI(TAG, "=== Single VL53L0X Sensor Example ===");
    
    // Create VL53L0X instance
    VL53L0X sensor;
    
    // Configure the sensor on I2C port 0, SCL=GPIO22, SDA=GPIO21
    if (!sensor.config(I2C_NUM_0, GPIO_NUM_22, GPIO_NUM_21)) {
        ESP_LOGE(TAG, "Failed to configure VL53L0X sensor");
        return;
    }
    
    // Initialize the sensor
    if (!sensor.init()) {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X sensor");
        return;
    }
    
    // Configure sensor settings
    sensor.setTimeout(500); // 500ms timeout
    sensor.setSignalRateLimit(0.1f); // Lower signal rate limit for longer range
    sensor.setMeasurementTimingBudget(200000); // 200ms timing budget
    
    ESP_LOGI(TAG, "VL53L0X sensor configured successfully");
    ESP_LOGI(TAG, "Sensor address: 0x%02X", sensor.getAddress());
    ESP_LOGI(TAG, "Timing budget: %lu us", sensor.getMeasurementTimingBudget());
    ESP_LOGI(TAG, "Signal rate limit: %.2f MCPS", sensor.getSignalRateLimit());
    
    // Take 10 single measurements
    for (int i = 0; i < 10; i++) {
        uint16_t distance = sensor.readRangeSingleMillimeters();
        
        if (sensor.timeoutOccurred()) {
            ESP_LOGW(TAG, "Measurement %d: TIMEOUT", i + 1);
        } else if (distance == 65535) {
            ESP_LOGW(TAG, "Measurement %d: OUT OF RANGE", i + 1);
        } else {
            ESP_LOGI(TAG, "Measurement %d: %d mm", i + 1, distance);
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait 1 second
    }
    
    ESP_LOGI(TAG, "Single sensor example completed");
}

void vl53l0x_example_multiple_sensors() {
    ESP_LOGI(TAG, "=== Multiple VL53L0X Sensors Example ===");
    
    // Create multiple VL53L0X instances
    VL53L0X sensor1, sensor2, sensor3;
    
    // Configure sensor 1 (default address 0x29)
    if (!sensor1.config(I2C_NUM_0, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_18)) {
        ESP_LOGE(TAG, "Failed to configure sensor 1");
        return;
    }
    
    // Configure sensor 2 with XSHUT control
    if (!sensor2.config(I2C_NUM_0, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_19)) {
        ESP_LOGE(TAG, "Failed to configure sensor 2");
        return;
    }
    
    // Configure sensor 3 with XSHUT control  
    if (!sensor3.config(I2C_NUM_0, GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_23)) {
        ESP_LOGE(TAG, "Failed to configure sensor 3");
        return;
    }
    
    // Initialize sensor 1 first (it will have default address 0x29)
    if (!sensor1.init()) {
        ESP_LOGE(TAG, "Failed to initialize sensor 1");
        return;
    }
    
    // Change sensor 1 address to 0x30
    sensor1.setAddress(0x30);
    ESP_LOGI(TAG, "Sensor 1 address changed to: 0x%02X", sensor1.getAddress());
    
    // Initialize sensor 2 and change its address
    if (!sensor2.init()) {
        ESP_LOGE(TAG, "Failed to initialize sensor 2");
        return;
    }
    sensor2.setAddress(0x31);
    ESP_LOGI(TAG, "Sensor 2 address changed to: 0x%02X", sensor2.getAddress());
    
    // Initialize sensor 3 and change its address
    if (!sensor3.init()) {
        ESP_LOGE(TAG, "Failed to initialize sensor 3");
        return;
    }
    sensor3.setAddress(0x32);
    ESP_LOGI(TAG, "Sensor 3 address changed to: 0x%02X", sensor3.getAddress());
    
    // Configure all sensors with same settings
    VL53L0X* sensors[] = {&sensor1, &sensor2, &sensor3};
    const char* names[] = {"Sensor1", "Sensor2", "Sensor3"};
    
    for (int i = 0; i < 3; i++) {
        sensors[i]->setTimeout(500);
        sensors[i]->setSignalRateLimit(0.1f);
        sensors[i]->setMeasurementTimingBudget(100000); // 100ms for faster multi-sensor reading
        ESP_LOGI(TAG, "%s configured", names[i]);
    }
    
    // Take measurements from all sensors
    for (int cycle = 0; cycle < 5; cycle++) {
        ESP_LOGI(TAG, "--- Measurement Cycle %d ---", cycle + 1);
        
        for (int i = 0; i < 3; i++) {
            uint16_t distance = sensors[i]->readRangeSingleMillimeters();
            
            if (sensors[i]->timeoutOccurred()) {
                ESP_LOGW(TAG, "%s: TIMEOUT", names[i]);
            } else if (distance == 65535) {
                ESP_LOGW(TAG, "%s: OUT OF RANGE", names[i]);
            } else {
                ESP_LOGI(TAG, "%s: %d mm", names[i], distance);
            }
        }
        
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait 2 seconds between cycles
    }
    
    ESP_LOGI(TAG, "Multiple sensors example completed");
}

void vl53l0x_example_continuous_mode() {
    ESP_LOGI(TAG, "=== Continuous Mode Example ===");
    
    VL53L0X sensor;
    
    // Configure and initialize sensor
    if (!sensor.config(I2C_NUM_0, GPIO_NUM_22, GPIO_NUM_21) || !sensor.init()) {
        ESP_LOGE(TAG, "Failed to configure/initialize sensor");
        return;
    }
    
    // Set up for continuous measurements
    sensor.setTimeout(500);
    sensor.setSignalRateLimit(0.25f);
    sensor.setMeasurementTimingBudget(50000); // 50ms for fast continuous measurements
    
    // Start continuous measurements with 100ms period
    sensor.startContinuous(100);
    ESP_LOGI(TAG, "Started continuous measurements (100ms period)");
    
    // Read continuous measurements for 10 seconds
    for (int i = 0; i < 100; i++) {
        uint16_t distance = sensor.readRangeContinuousMillimeters();
        
        if (sensor.timeoutOccurred()) {
            ESP_LOGW(TAG, "Continuous reading %d: TIMEOUT", i + 1);
        } else if (distance == 65535) {
            ESP_LOGW(TAG, "Continuous reading %d: OUT OF RANGE", i + 1);
        } else {
            ESP_LOGI(TAG, "Continuous reading %d: %d mm", i + 1, distance);
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS); // Match the measurement period
    }
    
    // Stop continuous measurements
    sensor.stopContinuous();
    ESP_LOGI(TAG, "Stopped continuous measurements");
    ESP_LOGI(TAG, "Continuous mode example completed");
}

// Main example function - call this from your app_main()
extern "C" void vl53l0x_examples() {
    ESP_LOGI(TAG, "Starting VL53L0X examples...");
    
    // Run single sensor example
    vl53l0x_example_single_sensor();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Run multiple sensors example (requires XSHUT control pins)
    // vl53l0x_example_multiple_sensors();
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Run continuous mode example
    vl53l0x_example_continuous_mode();
    
    ESP_LOGI(TAG, "All VL53L0X examples completed!");
}
