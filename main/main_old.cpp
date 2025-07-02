

// #include <stdio.h>

// #include "freeRTOS\freeRTOS.h"
// #include "freeRTOS\task.h"

// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
// #include "esp_log.h"

// #include "Motor.hpp"
// #include "ESP32Encoder.h"

// static const char* TAG = "MAIN";

// extern "C" void app_main() {
//     ESP32Encoder encoder;
//     // Motor motor;
//     // ESP_LOGI(TAG, "forward at 50%% speed");
//     // motor.set(true, 4096); // Forward at 50% speed
//     // vTaskDelay(2000 / portTICK_PERIOD_MS);

//     encoder.attachFullQuad(15,2);
//         // motor.set(false, 7000); // Reverse at 100% speed

//     vTaskDelay(5000 / portTICK_PERIOD_MS);
//         // motor.set(true, 7000); // Forward at 100% speed
//     while (true) {
//         ESP_LOGI(TAG, "Encoder count: %lld", encoder.getCount());

//         // motor.set(false, 7000); // Reverse at 100% speed
//         vTaskDelay(5000 / portTICK_PERIOD_MS);
//         ESP_LOGI(TAG, "Encoder count: %lld", encoder.getCount());

//         // motor.set(true, 7000); // Forward at 100% speed
//         vTaskDelay(5000 / portTICK_PERIOD_MS);

//     }
    
// }

