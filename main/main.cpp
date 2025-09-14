#include <stdio.h>

#include "freeRTOS\freeRTOS.h"
#include "freeRTOS\task.h"
#include "esp_heap_caps.h"
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
#include "Debouncer.hpp"
#include <cmath>

// #include "i2c.hpp"
// #include "logger.hpp"
// #include "timer.hpp"
// #include "vl53l.hpp"
#include "TCS34725.hpp"
#include "VL53L0X.hpp"
#include "LED.hpp"

#define POINT_DENSITY 200
// VL53L0X sensor;
TCS34725 colorSensor = TCS34725();
double ROBOT_WIDTH = 0.13; // in meters

static const char *TAG = "MAIN";

#define LINSPEED 0.02
#define signum(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))

enum class State
{
    OFFLINE,
    IDLE,
    ACTIVE_MOVING,
    ACTIVE_STOPPED,
    ACTIVE_WANDERING,
    ESCAPED_REVERSE,
    ESCAPED_ROTATE,
    REVERSE_THE_OTHER_WAY,
    CALIB_IMU
};
std::map<int, float> speedChanges;
float xPoses[POINT_DENSITY];
float yPoses[POINT_DENSITY];

ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

Motor motorLeft = Motor("left");
Motor motorRight = Motor("right");

VL53L0X sensorLeft;
VL53L0X sensorRight;

LED led;
PurePursuit purePursuit = PurePursuit(0.05); // Look ahead distance

State state = State::IDLE;
static int thresh;
static void runTheRobot(void *pvParameters)
{
    double yaw = IMUHelper::getYaw();
    std::map<double, std::pair<double, double>> distanceMap;
    int omegaChangeTimer = 0;
    const double omegaMax = 0.07;       // max turn rate (rad/s)
    const double omegaStep = 0.005;     // interpolation step
    const int omegaChangeInterval = 50; // change direction every 50 cycles (~15s if vTaskDelay is 300ms)
    int64_t reverseTimer = 0;
    int64_t rotateTimer = 0;
    int64_t rotateMaxTime = 0;
    bool rotateDirectionCcw = false;
    bool needToRotateTheOtherWay = false;
    Odometry::seed(Pose2d({0.0, 1, 0})); // Seed the odometry with a starting pose squircle/circle
    // Odometry::seed(Pose2d({2.01, 0.0, M_PI/2.0+0.01})); // Seed the odometry with a starting pose lisajous
    // Odometry::seed(Pose2d({0.0, 0.0, 0.0})); // Seed the odometry with a starting pose line

    Pose2d pose;

    // for (int i = 0; i < 200; ++i) {  // squircle
    //     float t = i/200.0*2*3.1415926; // t from 0 to 2*PI
    //     xPoses[i] = 1*signum(sin(t))*pow(abs(sin(t)),0.2);
    //     yPoses[i] = 1*signum(cos(t))*pow(abs(cos(t)),0.2);
    // }

    // for (int i = 0; i < 200; ++i) {  // lissajous figure
    //     float t = i/200.0*2*3.1415926; // t from 0 to 2*PI
    //     xPoses[i] = 2*sin(t+3.1415926/2);
    //     yPoses[i] = 1*sin(2*t);
    // }

    // LINE
    // for (int i = 0; i < 100; ++i) // line
    // {
    //     xPoses[i] = (i/100.0)*3;
    //     yPoses[i] = 0;
    // }
    // for (int i = 100; i < 200; ++i) // line
    // {
    //     xPoses[i] = ((200-i)/100.0)*3;
    //     yPoses[i] = 0;
    // }

    for (int i = 0; i < POINT_DENSITY; ++i) // 2 cycle circle
    {
        float t = i / (float)POINT_DENSITY * 2 * 3.1415926; // t from 0 to 2*PI
        xPoses[i] = 1 * sin(t);
        yPoses[i] = 1 * cos(t);
    }

    purePursuit.setPath(xPoses, yPoses, POINT_DENSITY);
    purePursuit.start();
    state = State::CALIB_IMU;
    while (true)
    {
        heap_caps_check_integrity_all(true);
        int reflectance = colorSensor.getClear();
        if (reflectance > thresh && state == State::ACTIVE_WANDERING)
        {
            state = State::ESCAPED_REVERSE;
            rotateTimer = 0;
            rotateMaxTime = 0;
            reverseTimer = esp_timer_get_time();
            ESP_LOGW(TAG, "Reflectance threshold exceeded! Reversing motors. %d", reflectance);
        }
        switch (state)
        {
        case State::REVERSE_THE_OTHER_WAY:
        {
            if (esp_timer_get_time() - rotateTimer < 3000000)
            {
                motorLeft.set(-0.5);
                motorRight.set(0.5);
            }
            else
            {
                state = State::ACTIVE_WANDERING;
                reverseTimer = 0;
                rotateTimer = 0;
                rotateMaxTime = 0;
            }
            led.set_color_rgb(0, 0, 255);
            break;
        }
        case State::ESCAPED_REVERSE:
        {
            // Reverse for 10 seconds
            if (esp_timer_get_time() - reverseTimer < 10000000)
            {
                motorLeft.set(-0.5);
                motorRight.set(-0.5);
            }
            else
            {
                state = State::ESCAPED_ROTATE;
                reverseTimer = 0;
                rotateTimer = 0;
                rotateMaxTime = 0;
                rotateTimer = esp_timer_get_time();
                // randomly generate a rotate time between 5 and 10 seconds
                rotateMaxTime = (rand() % 5000000) + 5000000; // in microseconds
                // rotateMaxTime = 5000000;
            }
            led.set_color_rgb(255, 0, 0);

            break;
        }
        case State::ESCAPED_ROTATE:
        {

            // Rotate in place for 1.5 seconds
            if (esp_timer_get_time() - rotateTimer < rotateMaxTime)
            {
                motorLeft.set(0.5);
                motorRight.set(-0.5);
                if (reflectance < thresh)
                {
                    state = State::REVERSE_THE_OTHER_WAY;
                    reverseTimer = 0;
                    rotateTimer = 0;
                    rotateMaxTime = 0;
                    rotateTimer = esp_timer_get_time();
                }
            }
            else
            {
                state = State::ACTIVE_WANDERING;
                reverseTimer = 0;
                rotateTimer = 0;
                rotateMaxTime = 0;
            }
            led.set_color_rgb(0, 0, 255);
            break;
        }
        case State::CALIB_IMU:
        {
            // Code for calibrating IMU
            led.set_color_rgb(255, 0, 0);
            motorLeft.set(1);
            motorRight.set(-1);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            motorLeft.stop();
            motorRight.stop();
            led.set_color_rgb(0, 255, 0);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            led.set_color_rgb(0, 0, 0);
            state = State::IDLE;
            Odometry::start();
            break;
        }
        case State::ACTIVE_WANDERING:
        {
            // Smooth random wandering
            static double omega = 0.0;
            static double targetOmega = 0.0;
            if (++omegaChangeTimer >= omegaChangeInterval)
            {
                targetOmega = ((rand() % 200) / 100.0 - 1.0) * omegaMax; // random value in [-omegaMax, omegaMax]
                omegaChangeTimer = 0;
            }
            // Smoothly interpolate omega toward targetOmega
            if (fabs(omega - targetOmega) > omegaStep)
            {
                omega += omegaStep * ((targetOmega > omega) ? 1 : -1);
            }
            else
            {
                omega = targetOmega;
            }
            double right = LINSPEED + ROBOT_WIDTH * omega / 2.0;
            double left = LINSPEED - ROBOT_WIDTH * omega / 2.0;
            // motorLeft.set(left / 0.025);
            // motorRight.set(right / 0.025);
            motorLeft.set(0.5);
            motorRight.set(0.5);
            ESP_LOGI(TAG, "WANDER: omega=%.5f target=%.5f left=%.5f right=%.5f", omega, targetOmega, left, right);
            led.set_color_rgb(0, 255, 0);
            break;
        }
        case State::ACTIVE_MOVING:
        { // Code for active moving state

            if (!purePursuit.checkStop())
            {
                pose = Odometry::getCurrentPose();
                purePursuit.compute(pose.getX(),
                                    pose.getY(),
                                    pose.getHeading());
                double omega = purePursuit.getCurvature() * LINSPEED; // m^-1 v_d = 0.01 m/s
                double right = LINSPEED + ROBOT_WIDTH * omega / 2.0;
                double left = LINSPEED - ROBOT_WIDTH * omega / 2.0;

                printf("Left: %f, Right: %f, Omega: %f, curvature: %f, X: %f, Y: %f, H: %f, Gx: %f, Gy: %f\n", left, right, omega, purePursuit.getCurvature(), pose.getX(), pose.getY(), pose.getHeading() * 180 / M_PI, purePursuit.getGoalX(), purePursuit.getGoalY());
                motorLeft.setReferenceMetersPerSec(left);
                motorRight.setReferenceMetersPerSec(right);
            }
            else
            {
                purePursuit = PurePursuit(0.05);
                purePursuit.setPath(xPoses, yPoses, 200);
                purePursuit.start();
                motorLeft.stop();
                motorRight.stop();
                ESP_LOGW(TAG, "Reached the goal!");
                led.set_color_rgb(255, 255, 0);
            }
            break;
        }
        case State::ACTIVE_STOPPED:
        {
            // STOPPPPPPPPP
            motorLeft.stop();
            motorRight.stop();
            break;
        }
        default:
        {
            break;
        }
            // ESP_LOGI(TAG, "Reflectance: %d State: %d, Left: %f, Right: %f, Reverse: %ld, Rotate: %ld", reflectance, (int)state, motorLeft.getOutput(), motorRight.getOutput(), reverseTimer, rotateTimer);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
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
    led.set_color_rgb(255, 255, 255);
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
        xTaskCreate(imuInitWrapper, "IMUInitTask", 8192, NULL, 5, &imuInitTask);
        while (!imu_init_done && elapsed < timeout_ms)
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            elapsed += 10;
            // If the task is deleted, init is done
            if (eTaskGetState(imuInitTask) == eDeleted)
            {
                imu_init_done = true;
                Odometry::setUseImu(true);
            }
        }
        if (!imu_init_done)
        {
            ESP_LOGE(TAG, "IMUHelper::init() timeout");
            Odometry::setUseImu(false);
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

    colorSensor.init(I2C_NUM_0, GPIO_NUM_5);

    encoderLeft.attachFullQuad(34, 35);
    encoderRight.attachFullQuad(36, 39);

    motorLeft.setup(GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_16, LEDC_TIMER_0, LEDC_CHANNEL_0, encoderLeft);
    motorRight.setup(GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_4, LEDC_TIMER_0, LEDC_CHANNEL_1, encoderRight);
    motorLeft.setPIDConstants(0.9549297, 0.047746485, 0.00, 0.24, 1.7);
    motorRight.setPIDConstants(0.9549297, 0.047746485, 0.00, 0.24, 1.7);

    motorLeft.testDirection();
    motorRight.testDirection();

    Odometry::setup(
        []()
        { return motorLeft.getMotorSpeedMetersPerSec(); },
        []()
        { return motorRight.getMotorSpeedMetersPerSec(); },
        []()
        { return IMUHelper::getYaw(); }, // Get yaw from IMU
        50                               // Update period in milliseconds
    );

    SettingsHelper::applySettings();
}

static void testEncoder()
{
    while (true)
    {
        motorLeft.set(0.5);
        motorRight.set(0.5);
        Pose2d p = Odometry::getCurrentPose();
        ESP_LOGI(TAG, "Left: %lld Right: %lld LS: %f RS %f, X: %f, Y: %f", encoderLeft.getCount(), encoderRight.getCount(), motorLeft.getMotorSpeed(), motorRight.getMotorSpeed(), p.getX(), p.getY());
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
extern "C" void app_main()
{

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    SettingsHelper::init();
    SNTPHelper::init();
    WifiHelper::init();
    vTaskDelay(5000 / portTICK_PERIOD_MS); // wait 5 second for wifi to init
    Telemetry::init();
    setupHardware();
    SNTPHelper::start();
    // SNTPHelper::waitForSync();
    SNTPHelper::print_current_time();
    Telemetry::waitForConnection();

    SettingsHelper::addDoubleSetting("robotWidth", ROBOT_WIDTH);
    ROBOT_WIDTH = SettingsHelper::getDoubleSetting("robotWidth");
    SettingsHelper::registerDoubleCallback("robotWidth", [](const std::pair<const char *, double> &setting)
                                           {
        ROBOT_WIDTH = setting.second;
        ESP_LOGI(TAG, "Set robot width to %f", ROBOT_WIDTH); });

    SubscriptionHandle handleCommand2 = Telemetry::subscribe("ledcolor", [](const char *topic, const char *data, int data_len)
                                                             {
        ESP_LOGI(TAG, "Received LED color command: %.*s", data_len, data);
        // Parse the color from the command
        int r = 0, g = 0, b = 0;
        sscanf(data, "[%d, %d, %d]", &r, &g, &b);
        led.set_color_rgb(r, g, b); });

    Odometry::seed(Pose2d({0.0, 0.0, 0.0})); // Seed the odometry with a starting pose

    // TaskHandle_t ledHandle;
    // xTaskCreate(do_led, "do_led", 4096, NULL, 1, &ledHandle);
    ESP_LOGI(TAG, "STARTING");

    SettingsHelper::addIntSetting("tcs/thresh", 150);
    thresh = SettingsHelper::getIntSetting("tcs/thresh");
    SettingsHelper::registerIntCallback("tcs/thresh", [](const std::pair<const char *, int> &p)
                                        {
        thresh = p.second;
        ESP_LOGI(TAG, "TCS threshold set to: %d", p.second); });

    uint8_t mac[6];
    char macString[13];
    esp_efuse_mac_get_default(mac);
    snprintf(macString, sizeof(macString), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("MAC Address: %s\n", macString);

    TaskHandle_t robotHandle;
    xTaskCreate(runTheRobot, "runTheRobot", 10240, NULL, 5, &robotHandle);
    SubscriptionHandle handleCommand = Telemetry::subscribe("command", [](const char *topic, const char *data, int data_len)
                                                            {
        ESP_LOGI(TAG, "Received command: %.*s", data_len, data);
        // Handle the command
        if (strncmp(data, "start", data_len) == 0)
        {
            ESP_LOGI(TAG, "Starting the robot");
            state = State::ACTIVE_WANDERING;
        }
        else if (strncmp(data, "stop", data_len) == 0)
        {
            ESP_LOGI(TAG, "Stopping the robot");
            motorLeft.stop();
            motorRight.stop();
            state = State::IDLE;
        }
        else if (strncmp(data, "respos", 6) == 0)
        {
            // Format: "respos,x,y,heading"
            float x = 0, y = 0, heading = 0;
            int parsed = sscanf(data, "respos,%f,%f,%f", &x, &y, &heading);
            if (parsed == 3) {
                ESP_LOGI(TAG, "Resetting odometry to x=%f, y=%f, heading=%f", x, y, heading);
                Odometry::seed(Pose2d(x, y, heading));
            } else {
                ESP_LOGW(TAG, "Malformed respos command: %.*s", data_len, data);
            }
        }
        else if (strncmp(data, "load_path", 9) == 0)
        {
            // Format: "load_pathAxyxyxy..." where A is int, x/y are floats
            if (data_len < 9 + sizeof(int)) {
                ESP_LOGW(TAG, "Malformed load_path command: too short");
            } else {
                // Pointer to start of packed data
                const char* ptr = data + 9;
                int n = 0;
                memcpy(&n, ptr, sizeof(int));
                ptr += sizeof(int);
                if (n > POINT_DENSITY) n = POINT_DENSITY;
                if (data_len < 9 + sizeof(int) + n * 2 * sizeof(float)) {
                    ESP_LOGW(TAG, "Malformed load_path command: not enough data for %d points", n);
                } else {
                    for (int i = 0; i < n; ++i) {
                        float x, y;
                        memcpy(&x, ptr, sizeof(float));
                        ptr += sizeof(float);
                        memcpy(&y, ptr, sizeof(float));
                        ptr += sizeof(float);
                        xPoses[i] = x;
                        yPoses[i] = y;
                    }
                    ESP_LOGI(TAG, "Loaded path with %d points. x[0]=%f, y[0]=%f", n, xPoses[0], yPoses[0]);
                    purePursuit.setPath(xPoses, yPoses, n);
                    purePursuit.start();
                    state = State::ACTIVE_MOVING;
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