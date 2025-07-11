// 

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
const double MAX_VEL = 0.05;     // in meters per second
const double MAX_ACCEL = 3.0;   // in meters per second per second
const double MAX_JERK = 6.0;    // in meters per second per second per second
const double ROBOT_WIDTH = 0.1; // in meters
const double WHEEL_RADIUS = 0.0254; // in meters
const double MAX_RPM = 6000; // in revolutions per minute

static const char* TAG = "MAIN";

// static void gyro_task(void* pvParameters)
// {
//     printf("Reading sensor data:\n");
   
// }


extern "C" void app_main() {
    // auto constraints = squiggles::Constraints(MAX_VEL, MAX_ACCEL, MAX_JERK);
    // auto generator = squiggles::SplineGenerator(
    // constraints,
    // std::make_shared<squiggles::TankModel>(ROBOT_WIDTH, constraints));

    // auto path = generator.generate({squiggles::Pose(0.0, 0.0, 1.0), squiggles::Pose(2.0, 2.0, 1.0)});

    
    
    // ESP32Encoder encoderLeft;
    ESP32Encoder encoderRight;
    // encoderLeft.attachFullQuad(34,35);
    encoderRight.attachFullQuad(36,39);

    Motor motorRight = Motor("RightMotor");
    motorRight.setup(GPIO_NUM_12, GPIO_NUM_13, 4, LEDC_TIMER_0, LEDC_CHANNEL_0);
    // motorLeft.setup(GPIO_NUM_14, GPIO_NUM_27, 16, LEDC_TIMER_1, LEDC_CHANNEL_1);


    motorRight.addEncoder(encoderRight);

    motorRight.setPIDConstants({.1, 0.05, 0.0, .1}); // Example PID constants
    // must be in order SNTP, MQTT, WiFi bc SNTP does the NVS/eventloop stuff
    // SNTPHelper::init();
    // SNTPHelper::setTimeZone("CST6CDT,M3.2.0,M11.1.0");
    // Telemetry::init();
    // WifiHelper::init();
    // SNTPHelper::start();
    // SNTPHelper::print_servers();

    // use the generated path velocities to control the motors
    // for (const auto& point : path) {
    //     ESP_LOGI(TAG, "Time: %f", point.time);
        
    //     ESP_LOGI(TAG, "Wheel Velocities: %f, %f", point.wheel_velocities[0], point.wheel_velocities[1]);
    //     // Set motor direction based on the velocity sign
    //     motorLeft.set(point.wheel_velocities[0] / MAX_VEL);
    //     motorRight.set(point.wheel_velocities[1] / MAX_VEL);
        
    //     vTaskDelay(static_cast<int>(100 / portTICK_PERIOD_MS));
    // }
    // motorRight.set(0.5);

    motorRight.setReferenceRpm(8);
    
    while (true) {

        vTaskDelay(250 / portTICK_PERIOD_MS);

    }
// motorRight.set(0.0);

    // read_gyro = true;

    // xTaskCreate(gyro_task, "gyroTask", 2048, NULL, 5, NULL);
}

// #include <stdio.h>
// #include "BNO08x.hpp"

// static const constexpr char* TAG = "Main";

// extern "C" void app_main(void)
// {
//     static BNO08x imu;

//     // initialize imu
//     if (!imu.initialize())
//     {
//         ESP_LOGE(TAG, "Init failure, returning from main.");
//         return;
//     }

//     // enable game rotation vector and calibrated gyro reports
//     imu.rpt.rv_game.enable(100000UL); // 100,000us == 100ms report interval
//     // imu.rpt.cal_gyro.enable(100000UL);
//     // imu.rpt.gravity.enable(100000UL);
//     // imu.rpt.accelerometer.enable(100000UL);
//     // imu.rpt.linear_accelerometer.enable(100000UL);

//     // see BNO08x::bno08x_reports_t for all possible reports to enable

//     // register callback to execute for all reports, 2 different methods

//     imu.dynamic_calibration_run_routine();
//     // method 1, void input param:
//     imu.register_cb(
//             []()
//             {
//                 if (imu.rpt.rv_game.has_new_data())
//                 {
//                     bno08x_euler_angle_t euler = imu.rpt.rv_game.get_euler();
//                     // ESP_LOGW(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler.x, euler.y, euler.z);
//                 }

//                 // if (imu.rpt.cal_gyro.has_new_data())
//                 // {
//                 //     bno08x_gyro_t velocity = imu.rpt.cal_gyro.get();
//                 //     ESP_LOGW(TAG, "Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s]", velocity.x, velocity.y, velocity.z);
//                 // }

//                 // if (imu.rpt.gravity.has_new_data())
//                 // {
//                 //     bno08x_accel_t grav = imu.rpt.gravity.get();
//                 //     ESP_LOGW(TAG, "Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2]", grav.x, grav.y, grav.z);
//                 // }

//                 // if (imu.rpt.accelerometer.has_new_data())
//                 // {
//                 //     bno08x_accel_t ang_accel = imu.rpt.accelerometer.get();
//                 //     ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
//                 // }

//                 // if (imu.rpt.linear_accelerometer.has_new_data())
//                 // {
//                 //     bno08x_accel_t lin_accel = imu.rpt.accelerometer.get();
//                 //     ESP_LOGW(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
//                 // }
//             });

//     // method 2, report ID param (comment method 1 out before commenting this in):
//     /*
//     imu.register_cb(
//             [](uint8_t rpt_ID)
//             {
//                 static bno08x_euler_angle_t euler;
//                 static bno08x_gyro_t velocity;
//                 static bno08x_accel_t grav;
//                 static bno08x_accel_t ang_accel;
//                 static bno08x_accel_t lin_accel;

//                 switch (rpt_ID)
//                 {
//                     case SH2_GAME_ROTATION_VECTOR:
//                         euler = imu.rpt.rv_game.get_euler();
//                         ESP_LOGW(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler.x, euler.y,
//                                 euler.z);
//                         break;

//                     case SH2_CAL_GYRO:
//                         velocity = imu.rpt.cal_gyro.get();
//                         ESP_LOGW(TAG, "Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s]", velocity.x, velocity.y, velocity.z);
//                         break;

//                     case SH2_GRAVITY:
//                         grav = imu.rpt.gravity.get();
//                         ESP_LOGW(TAG, "Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2]", grav.x, grav.y, grav.z);
//                         break;

//                     case SH2_ACCELEROMETER:
//                         ang_accel = imu.rpt.accelerometer.get();
//                         ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
//                         break;

//                     case SH2_LINEAR_ACCELERATION:
//                         lin_accel = imu.rpt.accelerometer.get();
//                         ESP_LOGW(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
//                         break;

//                     default:

//                         break;
//                 }
//             });

//     */

//     while (1)
//     {
//         // delay time is irrelevant, we just don't want to trip WDT
//         vTaskDelay(10000UL / portTICK_PERIOD_MS);
//     }
// }