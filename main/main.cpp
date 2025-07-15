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
#include "Odometry.hpp"
#include "PurePursuit.hpp"
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
    Odometry::seed(Pose2d({0.0,0.0,0.0})); // Seed the odometry with a starting pose
    Odometry::updateSimpleNoImu(0.05, 0.0, esp_timer_get_time());

    Pose2d pose;
    // double lastCurvature = 0.0;
            
        // double curvature = purePursuit.getCurvature();
        // double dcdt = (curvature - lastCurvature) / 0.050;
        // curvature = curvature - dcdt*0.001; // add damping
        
        // // print raw curvature, dcdt, corrected curvature, output curvature
        // printf("Curvature: %f, dcdt: %f, Corrected Curvature: %f", 
        //     purePursuit.getCurvature(), dcdt, curvature);
        // if (curvature > 100) {
        //     curvature = 100; // Limit curvature
        // } else if (curvature < -100) {
        //     curvature = -100; // Limit curvature
        // }

        // lastCurvature = curvature;
    while (true) {
        purePursuit.compute(pose.getX(),
                            pose.getY(),
                            pose.getHeading());
        double omega = purePursuit.getCurvature() * LINSPEED; // m^-1 v_d = 0.01 m/s
        // double right = (2*LINSPEED + ROBOT_WIDTH*omega)/2.0;
        // double left = 2*LINSPEED - right;
        // double omega = LINSPEED * 28.2842712; // should touch .035355339
        double right = LINSPEED + ROBOT_WIDTH*omega/2.0;
        double left = LINSPEED - ROBOT_WIDTH*omega/2.0;

        printf("Left: %f, Right: %f, Omega: %f, curvature: %f\n", left, right, omega, purePursuit.getCurvature());
        motorLeft.setReferenceMetersPerSec(left);
        motorRight.setReferenceMetersPerSec(right);
        // ESP_LOGI(TAG, "ticks: %llu", motorRight.getDistanceTicks());
        // Odometry::updateSimpleNoImu(left, right, esp_timer_get_time());
        Odometry::updateSimpleNoImu(motorLeft.getMotorSpeedMetersPerSec(), motorRight.getMotorSpeedMetersPerSec(), esp_timer_get_time());
        // ESP_LOGI(TAG, "left: %f, right: %f", motorLeft.getMotorSpeedMetersPerSec(), motorRight.getMotorSpeedMetersPerSec());
        pose = Odometry::getCurrentPose();

        

        vTaskDelay(50 / portTICK_PERIOD_MS); // Delay for 10 milliseconds
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

    // CALIBRATE KS
    // motorLeft.set(0.0);
    // motorRight.set(0.0);
    // double i = 0;
    // vTaskDelay(500/portTICK_PERIOD_MS);

    // ESP_LOGI(TAG, "LEFT");
    // while (!motorLeft.hasPower()) {
    //     i+=0.01;
    //     motorLeft.set(i);
    //     ESP_LOGI(TAG, "%f", i);
    //     vTaskDelay(500/portTICK_PERIOD_MS);
    // }
    // motorLeft.set(0.0);
    // motorRight.set(0.0);
    // i = 0;
    // vTaskDelay(500/portTICK_PERIOD_MS);
    // ESP_LOGI(TAG, "RIGHT");
    // while (!motorRight.hasPower()) {
    //     i+=0.01;
    //     motorRight.set(i);
    //     ESP_LOGI(TAG, "%f", i);
    //     vTaskDelay(500/portTICK_PERIOD_MS);
    // }

    // motorLeft.setReferenceMetersPerSec(0.039490);
    // motorRight.setReferenceRadPerSec(1.1967);

    TaskHandle_t handle;
    xTaskCreate(runTheRobot, "runTheRobot", 10240, NULL, 5, &handle);
    while (true) {
        // ESP_LOGI(TAG, "%d", uxTaskGetStackHighWaterMark(handle));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
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