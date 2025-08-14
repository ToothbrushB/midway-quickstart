

#include <stdio.h>

#include "freeRTOS\freeRTOS.h"
#include "freeRTOS\task.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "Motor.hpp"
#include "ESP32Encoder.h"
#include "driver\ledc.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/portmacro.h"
#include "sdkconfig.h"
#include "SNTPHelper.hpp"
#include "WifiHelper.hpp"
#include "Telemetry.hpp"
#include "Odometry.hpp"
#include "PurePursuit.hpp"
#include <IMUHelper.hpp>
#include <SettingsHelper.hpp>
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <chrono>
#include <vector>
#include <cmath>

// #include "i2c.hpp"
// #include "logger.hpp"
// #include "timer.hpp"
// #include "vl53l.hpp"
#include "TCS34725.hpp"
#include "LedStripHelper.hpp"
#include "VL53L0X.hpp"
#include "LED.hpp"

// VL53L0X sensor;
TCS34725 colorSensor = TCS34725();
double ROBOT_WIDTH = 0.13; // in meters

static const char *TAG = "MAIN";

#define LINSPEED 0.03

enum class State
{
    OFFLINE,
    IDLE,
    ACTIVE_MOVING,
    ACTIVE_STOPPED,
    OBSTRUCTED_STOPPED,
    OBSTRUCTED_REROUTING,
    OBSTRUCTED_SCANNING,
};
std::map<int, float> speedChanges;
float xPoses[200];
float yPoses[200];
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

Motor motorLeft = Motor("LeftMotor");
Motor motorRight = Motor("RightMotor");

VL53L0X sensorLeft;
VL53L0X sensorRight;

static void runTheRobot(void *pvParameters)
{
    double yaw = IMUHelper::getYaw();
    std::map<double, std::pair<double, double>> distanceMap;
    State state = State::ACTIVE_MOVING;
    switch (state)
    {
    case State::ACTIVE_MOVING:
    {                                                // Code for active moving state
        PurePursuit purePursuit = PurePursuit(0.05); // Look ahead distance
        purePursuit.setPath(xPoses, yPoses, 200);
        purePursuit.start();

        Odometry::seed(Pose2d({0.0, 0.0, IMUHelper::getYaw()})); // Seed the odometry with a starting pose

        Pose2d pose;
        while (!purePursuit.checkStop())
        {
            pose = Odometry::getCurrentPose();
            purePursuit.compute(pose.getX(),
                                pose.getY(),
                                pose.getHeading());
            double omega = purePursuit.getCurvature() * LINSPEED; // m^-1 v_d = 0.01 m/s
            double right = LINSPEED + ROBOT_WIDTH * omega / 2.0;
            double left = LINSPEED - ROBOT_WIDTH * omega / 2.0;

            printf("Left: %f, Right: %f, Omega: %f, curvature: %f\n", left, right, omega, purePursuit.getCurvature());
            motorLeft.setReferenceMetersPerSec(left);
            motorRight.setReferenceMetersPerSec(right);

            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        break;
    }
    case State::ACTIVE_STOPPED:
    {
        // STOPPPPPPPPP
        motorLeft.setReferenceMetersPerSec(0);
        motorRight.setReferenceMetersPerSec(0);
        break;
    }
    case State::OBSTRUCTED_STOPPED:
    {
        motorLeft.setReferenceMetersPerSec(0);
        motorRight.setReferenceMetersPerSec(0);
        // Check distances from sensors
        uint16_t distanceLeft = sensorLeft.readRangeContinuousMillimeters();
        uint16_t distanceRight = sensorRight.readRangeContinuousMillimeters();
        if (distanceLeft < ROBOT_WIDTH + 0.25 && distanceRight < ROBOT_WIDTH + 0.25)
        {
            state = State::ACTIVE_MOVING;
        }
        // Code for obstructed stopped state
        break;
    }
    case State::OBSTRUCTED_REROUTING:
    {
        // Code for obstructed rerouting state
        // For now, just stop the motors
        motorLeft.setReferenceMetersPerSec(0);
        motorRight.setReferenceMetersPerSec(0);
        // send message to the server requesting a reroute
        Telemetry::publishData("reroute_request", "true");
        // Wait for a response from the server
        while (state == State::OBSTRUCTED_REROUTING)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            // check for a new path from the server
        }
        break;
    }
    case State::OBSTRUCTED_SCANNING:
    {
        // Code for obstructed scanning state
        // spin turn
        motorLeft.setReferenceMetersPerSec(0.1);
        motorRight.setReferenceMetersPerSec(-0.1);
        // Scan the environment using the sensors
        uint16_t distanceLeftScan = sensorLeft.readRangeContinuousMillimeters();
        uint16_t distanceRightScan = sensorRight.readRangeContinuousMillimeters();

        // Record the distances at different yaw angles
        double tempYaw = IMUHelper::getYaw();    // Convert to radians
        double yawRad = fmod(tempYaw, 2 * M_PI); // Normalize to [0, 2π)

        // Record distances every 5 degrees (converted to radians)
        if (std::fmod(tempYaw, 1.0 * M_PI / 180.0) < 0.01)
        {
            distanceMap[tempYaw] = {distanceLeftScan, distanceRightScan};
        }

        // Check if we've completed a full rotation (2π radians)
        if (std::abs(tempYaw - (yawRad + 2 * M_PI)) < 0.1)
        {
            // If we have completed a full rotation, stop the motors
            motorLeft.setReferenceMetersPerSec(0);
            motorRight.setReferenceMetersPerSec(0);
        }
        ESP_LOGI(TAG, "Left Distance: %d mm, Right Distance: %d mm", distanceLeftScan, distanceRightScan);
        break;
    }
    default:
    {
        break;
    }
    }

    // for (int i = 0; i < 200; ++i) {
    //     // float t = i/200.0*2*3.1415926; // t from 0 to 2*PI
    //     // xPoses[i] = cos(t)*sin(3*t);
    //     // yPoses[i] = sin(t)*sin(3*t);
    //     xPoses[i] =  i/200.0; // Linear path in x direction
    //     yPoses[i] = i/200.0; // Linear path in y direction
    // }

    // // publish entire xPoses and yPoses into MQTT
    // for (size_t i = 0; i < 200; ++i) {
    //     Telemetry::publishData("path", std::to_string(xPoses[i]) + ", " + std::to_string(yPoses[i]));
    // }
}

extern "C" void app_main()
{

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    SettingsHelper::init();
    // SNTPHelper::init();
    // WifiHelper::init();
    // Telemetry::init();
    // SNTPHelper::start();
    // SNTPHelper::waitForSync();
    // SNTPHelper::print_current_time();
    // Telemetry::waitForConnection();

    LED led;
    led.init();
    led.set_color_rgb(0, 255, 0); // Green to indicate system is starting

    // Use ledc to control an LED connected to GPIO 33
    // ledc_timer_config_t ledc_timer = {
    //     .speed_mode = LEDC_HIGH_SPEED_MODE, // Use high speed mode
    //     .duty_resolution = LEDC_TIMER_13_BIT, // St duty resolution to 13 bits
    //     .timer_num = LEDC_TIMER_0, // Use timer 0
    //     .freq_hz = 5000, // Set frequency to 5 kHz
    //     .clk_cfg = LEDC_AUTO_CLK,
    //     .deconfigure = false
    // };
    // ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    // ledc_channel_config_t ledc_channel = {
    //     .gpio_num = GPIO_NUM_33, // Connect to GPIO 33
    //     .speed_mode = LEDC_HIGH_SPEED_MODE, // Use high speed mode
    //     .channel = LEDC_CHANNEL_0, // Use channel 0
    //     .intr_type = LEDC_INTR_DISABLE,
    //     .timer_sel = LEDC_TIMER_0, // Use timer 0
    //     .duty = 0, // Set initial duty to 0
    //     .hpoint = 0,
    //     .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
    //     .flags = {
    //         .output_invert = 0
    //     }
    // };
    // ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    // ESP_LOGI(TAG, "=== Robot Initialization ===");
    // // set LED d to 50% duty cycle
    // ESP_ERROR_CHECK(ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 6454)); // 50% duty cycle for 13-bit resolution
    // ESP_ERROR_CHECK(ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel)); // 6454 ~= 2.6 // 5461 ~= 2.2

    encoderLeft.attachFullQuad(34, 35);
    encoderRight.attachFullQuad(36, 39);
    motorLeft.setup(GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_16, LEDC_TIMER_0, LEDC_CHANNEL_0, encoderLeft);
    motorRight.setup(GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_4, LEDC_TIMER_0, LEDC_CHANNEL_1, encoderRight);
    // SettingsHelper::addDoubleSetting("robotWidth", ROBOT_WIDTH);
    // ROBOT_WIDTH = SettingsHelper::getDoubleSetting("robotWidth");
    // SettingsHelper::registerDoubleCallback("robotWidth", [](const std::pair<const char *, double> &setting)
                                        //    {
        // ROBOT_WIDTH = setting.second;
        // ESP_LOGI(TAG, "Set robot width to %f", ROBOT_WIDTH); });
    // motorLeft.set(0.5);
    // motorRight.set(0.5);                                    
    // sensorLeft.config(I2C_NUM_0, 22, 21, 18, 0x29, 0);
    // sensorRight.config(I2C_NUM_0, 22, 21, 19, 0x29, 0);

    // // Initialize the sensor

    // vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for sensor to initialize
    // sensorLeft.init(0x31);
    // ESP_LOGW(TAG, "SENSOR 2");
    // sensorRight.init(0x30);
    // ESP_LOGW("TAG", "PLUG IN COLOR SENSOR");
    // vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait for sensor to initialize
    // // colorSensor.init(I2C_NUM_0);
    // // Configure sensor settings
    // sensorLeft.setTimeout(500);                    // 500ms timeout
    // sensorLeft.setSignalRateLimit(0.1f);           // Lower signal rate limit for longer range
    // sensorLeft.setMeasurementTimingBudget(200000); // 200ms timing budget

    // ESP_LOGI(TAG, "VL53L0X sensor configured successfully");
    // ESP_LOGI(TAG, "Sensor address: 0x%02X", sensorLeft.getAddress());
    // ESP_LOGI(TAG, "Timing budget: %lu us", sensorLeft.getMeasurementTimingBudget());
    // ESP_LOGI(TAG, "Signal rate limit: %.2f MCPS", sensorLeft.getSignalRateLimit());
    // sensorLeft.startContinuous(100);
    // sensorRight.startContinuous(100);
    // ESP_LOGI(TAG, "Started continuous measurements (100ms period)");

    // //     // Read continuous measurements for 10 seconds
    // // while (1) {
    // //     uint16_t distanceLeft = sensorLeft.readRangeContinuousMillimeters();
    // //     uint16_t distanceRight = sensorRight.readRangeContinuousMillimeters();
    // // }
    // // uint16_t distance2=0;
    // //     Color color = colorSensor.getColor();
    // //   int r = gammatable[color.r];
    // //   int g = gammatable[color.g];
    // //   int b = gammatable[color.b];

    // //     // IMUHelper::init();
    // //     // IMUHelper::calibrate();
    // //     // IMUHelper::start();
    // //     printf("1\n");

    // Odometry::setup(
    //     []()
    //     { return motorLeft.getMotorSpeedMetersPerSec(); },
    //     []()
    //     { return motorRight.getMotorSpeedMetersPerSec(); },
    //     []()
    //     { return IMUHelper::getYaw(); }, // Get yaw from IMU
    //     50                               // Update period in milliseconds
    // );
    // SubscriptionHandle handle = Telemetry::subscribe("set_path", [](const char *topic, const char *data, int data_len)
    //                                                  {
    //     ESP_LOGI(TAG, "Received path data: %s", data);
    //     // Parse the path data and set it in PurePursuit
    //     std::vector<float> xPoses;
    //     std::vector<float> yPoses;
    //     std::istringstream ss(data);
    //     std::string token;
    //     while (std::getline(ss, token, '\n')) {
    //         // Split the token based on delimiter comma
    //         std::istringstream tokenStream(token);
    //         std::string xStr, yStr, speedStr;
    //         if (std::getline(tokenStream, xStr, ',') && std::getline(tokenStream, yStr, ',')) {
    //             xPoses.push_back(std::stof(xStr));
    //             yPoses.push_back(std::stof(yStr));
    //             ESP_LOGI(TAG, "Parsed pose: (%f, %f)", std::stof(xStr), std::stof(yStr));
    //         }
    //         if (std::getline(tokenStream, speedStr, ',')) {
    //             speedChanges.insert({xPoses.size() - 1, std::stof(speedStr)});
    //             ESP_LOGI(TAG, "Parsed speed: %f", std::stof(speedStr));
    //         }
    //     } });

    // SubscriptionHandle handleStart = Telemetry::subscribe("start", [](const char *topic, const char *data, int data_len)
    //                                                       {
    //     TaskHandle_t robotHandle;
    //     xTaskCreate(runTheRobot, "runTheRobot", 10240, NULL, 5, &robotHandle); });

    // motorRight.set(1);
    // motorLeft.set(1);
    // while (1) {
    //   double r = motorRight.getMotorSpeed();
    //   double l = motorLeft.getMotorSpeed();
    //   printf("Motor Right Speed: %f, Motor Left Speed: %f\n", r, l);
    //   vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // ESP_LOGI(TAG, "=== Single VL53L0X Sensor Example ===");

    // Create VL53L0X instance
    // VL53L0X sensor;
    // VL53L0X sensor2;
    // colorSensor.init(I2C_NUM_0);
    // Configure the sensor on I2C port 0, SCL=GPIO22, SDA=GPIO21

    //       HSVColor hsv = LedStripHelper::rgb2hsv({static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)});
    //       if (distance < 400) {
    //         LedStripHelper::set_color_hsv(hsv.hue, hsv.saturation, hsv.value/(400-distance));
    //       }
    //       else {
    //         LedStripHelper::off();
    //       }
    //         printf("Distance: %d mm\tDistance2: %d mm\n", distance, distance2);

    //         vTaskDelay(pdMS_TO_TICKS(100));
    //     }

    // ESP_LOGI(TAG, "Single sensor example completed");
}