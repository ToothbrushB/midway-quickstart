

#include <stdio.h>

#include "freeRTOS\freeRTOS.h"
#include "freeRTOS\task.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "Motor.hpp"
#include "ESP32Encoder.h"
#include "driver\ledc.h"
#include "squiggles.hpp"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

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
const double MAX_VEL = 0.05;     // in meters per second
const double MAX_ACCEL = 3.0;   // in meters per second per second
const double MAX_JERK = 6.0;    // in meters per second per second per second
const double ROBOT_WIDTH = 0.13; // in meters
const double WHEEL_RADIUS = 0.033; // in meters

const double SCALE_FACTOR = 50; // Scale factor for curvature to match the robot's scale
static const char* TAG = "MAIN";

#define LINSPEED 0.03
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

Motor motorLeft = Motor("LeftMotor");
Motor motorRight = Motor("RightMotor");
static void runTheRobot(void* pvParameters)
{
//     ESP_LOGI(TAG, "Free heap before path generation: %lu bytes", esp_get_free_heap_size());
//     ESP_LOGI(TAG, "Minimum free heap: %lu bytes", esp_get_minimum_free_heap_size());

//    auto constraints = squiggles::Constraints(MAX_VEL, MAX_ACCEL, MAX_JERK);
//     auto generator = squiggles::SplineGenerator(
//     constraints,
//     std::make_shared<squiggles::TankModel>(ROBOT_WIDTH, constraints),
//     0.5); // 0.5 seconds between path points

//     auto path = generator.generate({squiggles::Pose(0.0, 0.0, 0.0), squiggles::Pose(2.0, 0.0, 0.0)});
    

    
    // collect path[i].vector.pose.x into an array float* xPoses
    float xPoses[200];
    float yPoses[200];
    for (int i = 0; i < 200; ++i) {
        float t = i/200.0*2*3.1415926; // t from 0 to 2*PI
        xPoses[i] = cos(t)*sin(3*t); 
        yPoses[i] = sin(t)*sin(3*t);
    }

    // publish entire xPoses and yPoses into MQTT
    for (size_t i = 0; i < 200; ++i) {
        Telemetry::publishData("path", std::to_string(xPoses[i]) + ", " + std::to_string(yPoses[i]));
    }

    PurePursuit purePursuit = PurePursuit(0.05); // Look ahead distance
    purePursuit.setPath(xPoses, yPoses, 200);
    purePursuit.start();



    // motorRight.set(1);
    Odometry::seed(Pose2d({0.0,0.0,IMUHelper::getYaw()})); // Seed the odometry with a starting pose
    // Odometry::updateSimpleNoImu(0.05, 0.0, esp_timer_get_time());

    Pose2d pose;
    while (!purePursuit.checkStop()) {
        purePursuit.compute(pose.getX(),
                            pose.getY(),
                            pose.getHeading());
        double omega = purePursuit.getCurvature() * LINSPEED; // m^-1 v_d = 0.01 m/s
        double right = LINSPEED + ROBOT_WIDTH*omega/2.0;
        double left = LINSPEED - ROBOT_WIDTH*omega/2.0;

        printf("Left: %f, Right: %f, Omega: %f, curvature: %f\n", left, right, omega, purePursuit.getCurvature());
        motorLeft.setReferenceMetersPerSec(left);
        motorRight.setReferenceMetersPerSec(right);

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    // for (const auto& point : path) {
    //     // ESP_LOGI(TAG, "Time: %f", point.time);
        
    //     // ESP_LOGI(TAG, "Wheel Velocities: %f, %f", point.wheel_velocities[0], point.wheel_velocities[1]); // meters per sec
    //     // Set motor direction based on the velocity sign
    //     motorLeft.setReferenceMetersPerSec(point.wheel_velocities[0]);
    //     motorRight.setReferenceMetersPerSec(point.wheel_velocities[1]);
        
    //     vTaskDelay(static_cast<int>(100 / portTICK_PERIOD_MS));
    // }

}



extern "C" void app_main() {
    

    // must be in order SNTP, WiFi, MQTT bc SNTP does the NVS/eventloop stuff
    SNTPHelper::init();
    SNTPHelper::setTimeZone("CST6CDT,M3.2.0,M11.1.0");
    WifiHelper::init();
    Telemetry::init();
    Telemetry::waitForConnection();
    SNTPHelper::start();
    
    encoderLeft.attachFullQuad(34,35);
    encoderRight.attachFullQuad(36,39);

    motorLeft.setup(GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_16, LEDC_TIMER_0, LEDC_CHANNEL_0);
    motorRight.setup(GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_4, LEDC_TIMER_1, LEDC_CHANNEL_1);


    motorRight.addEncoder(encoderRight);
    motorLeft.addEncoder(encoderLeft);
    motorRight.setPIDConstants({
        .kP = .9549297, 
        .kI = 0.47746485, 
        .kD = 0.0, 
        .kS = 0.24,
        .kV = 1.7 //.9549297
    }); // Example PID constants
    motorLeft.setPIDConstants({
        .kP = .9549297, 
        .kI = 0.47746485, 
        .kD = 0.0, 
        .kS = 0.3,
        .kV = 1.7 //.9549297
    }); // Example PID constants

    IMUHelper::init();
    // IMUHelper::calibrate();
    IMUHelper::start();


    Odometry::setup(
        []() { return motorLeft.getMotorSpeedMetersPerSec(); },
        []() { return motorRight.getMotorSpeedMetersPerSec(); },
        []() { return IMUHelper::getYaw(); }, // Get yaw from IMU
        50 // Update period in milliseconds
    );


    SubscriptionHandle handle = Telemetry::subscribe("start", [](const char* topic, const char* data, int data_len) {
            TaskHandle_t robotHandle;
        xTaskCreate(runTheRobot, "runTheRobot", 10240, NULL, 5, &robotHandle);
    });


    while (true) {
        // ESP_LOGI(TAG, "%d", uxTaskGetStackHighWaterMark(robotHandle));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}