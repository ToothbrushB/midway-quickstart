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

Motor motorLeft = Motor("left");
Motor motorRight = Motor("right");

VL53L0X sensorLeft;
VL53L0X sensorRight;

LED led;

State state = State::IDLE;

static void runTheRobot(void *pvParameters)
{
    double yaw = IMUHelper::getYaw();
    std::map<double, std::pair<double, double>> distanceMap;

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
        Telemetry::publishData("reroute_request", "{\"reroute\": true}");
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

void do_led(void *pvParameters)
{
    int red[] = {100, 230, 25, 30, 30, 50, 50, 100, 255, 255, 255, 255};
    int green[] = {100, 100, 15, 10, 30, 15, 20, 20, 165, 0, 25, 50};
    int blue[] = {100, 200, 40, 20, 30, 30, 80, 20, 50, 0, 0, 0};

    while (true)
    {

        // pick a random color from the above arrays
        int index = rand() % 6;
        led.set_color_rgb(red[index], green[index], blue[index]);
        ESP_LOGI(TAG, "LED ON%d R:%d G:%d B:%d", index, red[index], green[index], blue[index]);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // ESP_LOGI(TAG, "yaw: %.2f", IMUHelper::getYaw(true));

        led.set_color_rgb(0, 0, 0);
        ESP_LOGI(TAG, "LED OFF");

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void setupHardware()
{
    led.init();

    // Add timeout for IMUHelper::init()
    {
        const int timeout_ms = 2000;
        int elapsed = 0;
        bool imu_init_done = false;
        TaskHandle_t imuInitTask;
        auto imuInitWrapper = [](void *param)
        {
            IMUHelper::init();
            IMUHelper::start();
            vTaskDelete(NULL);
        };
        xTaskCreate(imuInitWrapper, "IMUInitTask", 2048, NULL, 5, &imuInitTask);
        while (!imu_init_done && elapsed < timeout_ms)
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            elapsed += 10;
            // If the task is deleted, init is done
            if (eTaskGetState(imuInitTask) == eDeleted)
            {
                imu_init_done = true;
            }
        }
        if (!imu_init_done)
        {
            ESP_LOGE(TAG, "IMUHelper::init() timeout");
            vTaskDelete(imuInitTask);
        }
    }

    if (i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK)
    {
        ESP_LOGI(TAG, "I2C driver installed successfully");
        i2c_config_t config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = 21,
            .scl_io_num = 22,
            .sda_pullup_en = true,
            .scl_pullup_en = true,
            .master = {.clk_speed = 100000}};
        if (i2c_param_config(I2C_NUM_0, &config))
        { // Config failed
            i2c_driver_delete(I2C_NUM_0);
            ESP_LOGE(TAG, "I2C param config failed");
        }
        else
        {
            ESP_LOGI(TAG, "I2C param config successful");
        }
        i2c_set_timeout(I2C_NUM_0, 80000); // Clock stretching
        i2c_filter_enable(I2C_NUM_0, 5);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to install I2C driver");
    }

    colorSensor.init(I2C_NUM_0);

    encoderLeft.attachFullQuad(34, 35);
    encoderRight.attachFullQuad(36, 39);

    motorLeft.setup(GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_16, LEDC_TIMER_0, LEDC_CHANNEL_0, encoderLeft);
    motorRight.setup(GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_4, LEDC_TIMER_0, LEDC_CHANNEL_1, encoderRight);
    motorLeft.setPIDConstants(0.9549297, 0.047746485, 0.00, 0.24, 1.7);
    motorRight.setPIDConstants(0.9549297, 0.047746485, 0.00, 0.24, 1.7);
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
    setupHardware();
    SettingsHelper::addDoubleSetting("robotWidth", ROBOT_WIDTH);
    ROBOT_WIDTH = SettingsHelper::getDoubleSetting("robotWidth");
    SettingsHelper::registerDoubleCallback("robotWidth", [](const std::pair<const char *, double> &setting)
                                           {
        ROBOT_WIDTH = setting.second;
        ESP_LOGI(TAG, "Set robot width to %f", ROBOT_WIDTH); });

    Odometry::setup(
        []()
        { return motorLeft.getMotorSpeedMetersPerSec(); },
        []()
        { return motorRight.getMotorSpeedMetersPerSec(); },
        []()
        { return IMUHelper::getYaw(); }, // Get yaw from IMU
        50                               // Update period in milliseconds
    );
    SubscriptionHandle handleCommand2 = Telemetry::subscribe("ledcolor", [](const char *topic, const char *data, int data_len)
                                                             {
        ESP_LOGI(TAG, "Received LED color command: %.*s", data_len, data);
        // Parse the color from the command
        int r = 0, g = 0, b = 0;
        sscanf(data, "[%d, %d, %d]", &r, &g, &b);
        led.set_color_rgb(r, g, b); });

    // PurePursuit purePursuit = PurePursuit(0.05); // Look ahead distance
    // purePursuit.setPath(xPoses, yPoses, 200);
    // purePursuit.start();

    Odometry::seed(Pose2d({0.0, 0.0, IMUHelper::getYaw()})); // Seed the odometry with a starting pose

    TaskHandle_t ledHandle;
    xTaskCreate(do_led, "do_led", 4096, NULL, 1, &ledHandle);
    Pose2d pose;
    // while (!purePursuit.checkStop())
    ESP_LOGI(TAG, "STARTING");

    motorLeft.testDirection();
    motorRight.testDirection();
    SettingsHelper::applySettings();

    SettingsHelper::addIntSetting("tcs/thresh", 200);
    int thresh = SettingsHelper::getIntSetting("tcs/thresh");
    SettingsHelper::registerIntCallback("tcs/thresh", [&thresh](const std::pair<const char*, int>& p)
                                         {
        thresh = p.second;
        ESP_LOGI(TAG, "TCS threshold set to: %d", p.second);
    });
    
    while (true)
    {
        int reflectance = colorSensor.getClear();
        if (reflectance > thresh) {
            motorLeft.setReferenceMetersPerSec(0);
            motorRight.setReferenceMetersPerSec(0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        pose = Odometry::getCurrentPose();

        // purePursuit.compute(pose.getX(),
        //                     pose.getY(),
        //                     pose.getHeading());
        // double omega = LINSPEED / 1;
        // double omega = purePursuit.getCurvature() * LINSPEED; // m^-1 v_d = 0.01 m/s
        // double right = LINSPEED + ROBOT_WIDTH * omega / 2.0;
        // double left = LINSPEED - ROBOT_WIDTH * omega / 2.0;

        // printf("Left: %f, Right: %f, Omega: %f\n", left, right, omega);
        motorLeft.setReferenceMetersPerSec(0.02);
        motorRight.setReferenceMetersPerSec(0.02);

        double speedRight = motorRight.getMotorSpeedMetersPerSec();
        double speedLeft = motorLeft.getMotorSpeedMetersPerSec();

        printf("leftset: %f, rightset: %f, left: %f, right: %f\n", 0.02, 0.02, speedLeft, speedRight);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }

    SubscriptionHandle handleCommand = Telemetry::subscribe("command", [](const char *topic, const char *data, int data_len)
                                                            {
        ESP_LOGI(TAG, "Received command: %.*s", data_len, data);
        // Handle the command
        if (strncmp(data, "start", data_len) == 0)
        {
            ESP_LOGI(TAG, "Starting the robot");
            TaskHandle_t robotHandle;
            xTaskCreate(runTheRobot, "runTheRobot", 10240, NULL, 5, &robotHandle);
            state = State::ACTIVE_MOVING;
        }
        else if (strncmp(data, "stop", data_len) == 0)
        {
            ESP_LOGI(TAG, "Stopping the robot");
            motorLeft.setReferenceMetersPerSec(0);
            motorRight.setReferenceMetersPerSec(0);
            state = State::IDLE;
        }
        else if (strncmp(data, "reset_odometry", data_len) == 0)
        {
            ESP_LOGI(TAG, "Resetting odometry");
            Odometry::seed(Pose2d({0.0, 0.0, IMUHelper::getYaw()}));
        }
        else if (strncmp(data, "load_path", 9) == 0)
        {
            ESP_LOGI(TAG, "Loading path");
            // Load the path data
            // command should be structured like "load_path, x[1, 2, 5, 3...], y[1, 2, 3, 1...]"
            for (int i = 11; i < data_len; i++)
            {
                if (data[i] == 'x' && data[i + 1] == '[')
                {
                    i += 2;
                    int j = 0;
                    while (data[i] != ']' && i < data_len)
                    {
                        xPoses[j++] = atof(&data[i]);
                        while (data[i] != ',' && data[i] != ']' && i < data_len) {
                            i++;
                        }
                    }
                }
                else if (data[i] == 'y' && data[i + 1] == '[')
                {
                    i += 2;
                    int j = 0;
                    while (data[i] != ']' && i < data_len)
                    {
                        yPoses[j++] = atof(&data[i]);
                        while (data[i] != ',' && data[i] != ']' && i < data_len) {
                            i++;
                        }
                    }
                }
            }
        }
        else if (strncmp(data, "blink_pattern", 13) == 0)
        {
            std::vector<Color> colors;

            ESP_LOGI(TAG, "Loading blink pattern");
            // Load the blink pattern data
            // command should be structured like "blink_pattern, [128, 255, 0], [1, 2, 48], [89, 89, 89]..."
            for (int i = 15; i < data_len; i++)
            {
                if (data[i] == '[')
                {
                    i++;
                    int j = 0;  
                    if (i < data_len) // could run into out of bounds error if not formatted correctly
                    {
                        int r = 0, g = 0, b = 0;
                        sscanf(&data[i], "[%d, %d, %d]", &r, &g, &b);
                        colors.push_back(Color{static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)});
                    }
                }
            }
            led.change_blink_pattern(colors);
        }
        else
        {
            ESP_LOGW(TAG, "Unknown command: %.*s", data_len, data);
        } });
}