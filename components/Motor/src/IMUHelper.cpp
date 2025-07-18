#include "IMUHelper.hpp"
#include "BNO08x.hpp"
#include "esp_log.h"
#include "Telemetry.hpp"
#define RAD_2_DEG 57.29577951308232f // 180 / PI
#define DEG_2_RAD 0.017453292519943295f // PI / 180
static const char* TAG = "IMUHelper";
BNO08x IMUHelper::imu;
bno08x_euler_angle_t IMUHelper::euler;
void IMUHelper::calibrate() {
    imu.dynamic_calibration_run_routine();
}
void IMUHelper::init() {
    if (!imu.initialize()) {
        ESP_LOGE("IMUHelper", "Failed to initialize IMU");
    } else {
        ESP_LOGI("IMUHelper", "IMU initialized successfully");
    }

}
void IMUHelper::tare() {
    imu.rpt.rv.tare_clear();
    imu.rpt.rv.tare(true, true, true);
    imu.rpt.rv.tare_persist();
    // imu.rpt.rv_game.tare(true, true, true);
    // imu.rpt.rv_geomagnetic.tare(true, true, true);
}
void IMUHelper::start() {
    imu.rpt.rv.enable(100000UL); // 1,000us == 1ms report interval
    // imu.rpt.cal_gyro.enable(100000UL);
    // imu.rpt.gravity.enable(100000UL);
    // imu.rpt.accelerometer.enable(100000UL);
    // imu.rpt.linear_accelerometer.enable(100000UL);

    // see BNO08x::bno08x_reports_t for all possible reports to enable

    // register callback to execute for all reports, 2 different methods

    // method 2, report ID param (comment method 1 out before commenting this in):
    imu.register_cb(
            [](uint8_t rpt_ID)
            {
                switch (rpt_ID)
                {
                    case SH2_ROTATION_VECTOR:
                        euler = imu.rpt.rv.get_euler(false);
                        if (euler.rad_accuracy < 10*DEG_2_RAD) {
                            imu.dynamic_calibration_disable(BNO08xCalSel::all);
                        }
                        Telemetry::publishData("imu", IMUHelper::euler.toString());
                        // ESP_LOGW(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg] accuracy: %.2f", euler.x*RAD_2_DEG, euler.y*RAD_2_DEG,
                                // euler.z*RAD_2_DEG, euler.rad_accuracy*RAD_2_DEG);
                        break;

                    // case SH2_CAL_GYRO:
                    //     velocity = imu.rpt.cal_gyro.get();
                    //     ESP_LOGW(TAG, "Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s]", velocity.x, velocity.y, velocity.z);
                    //     break;

                    // case SH2_GRAVITY:
                    //     grav = imu.rpt.gravity.get();
                    //     ESP_LOGW(TAG, "Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2]", grav.x, grav.y, grav.z);
                    //     break;

                    // case SH2_ACCELEROMETER:
                    //     ang_accel = imu.rpt.accelerometer.get();
                    //     ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
                    //     break;

                    // case SH2_LINEAR_ACCELERATION:
                    //     lin_accel = imu.rpt.accelerometer.get();
                    //     ESP_LOGW(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
                    //     break;

                    default:

                        break;
                }
            });
}

double IMUHelper::getYaw() {
    return IMUHelper::euler.z;
}