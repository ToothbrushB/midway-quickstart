#include "IMUHelper.hpp"
#include "BNO08x.hpp"
#include "esp_log.h"
#include "Telemetry.hpp"
#include "SettingsHelper.hpp"
#define RAD_2_DEG 57.29577951308232f    // 180 / PI
#define DEG_2_RAD 0.017453292519943295f // PI / 180
static const char *TAG = "IMUHelper";
BNO08x IMUHelper::imu;
bno08x_euler_angle_t IMUHelper::euler;
double IMUHelper::yawOffset = 0.0; // Initialize yaw offset
bool IMUHelper::reverseYaw = false;

void IMUHelper::resetYaw()
{
    yawOffset = IMUHelper::euler.z + yawOffset; // Reset the yaw offset to the current yaw
    ESP_LOGI(TAG, "Yaw offset reset to: %.2f", yawOffset);
}
void IMUHelper::calibrate()
{
    imu.dynamic_calibration_run_routine();
}
void IMUHelper::init()
{
    if (!imu.initialize())
    {
        ESP_LOGE("IMUHelper", "Failed to initialize IMU");
    }
    else
    {
        ESP_LOGI("IMUHelper", "IMU initialized successfully");
    }
    SettingsHelper::addBoolSetting("imu/rev", false);
    reverseYaw = SettingsHelper::getBoolSetting("imu/rev");
    SettingsHelper::registerBoolCallback("imu/rev", [](const std::pair<const char*, bool>& p)
                                         {
        IMUHelper::reverseYaw = p.second;
        ESP_LOGI(TAG, "IMU reverse yaw set to: %s", IMUHelper::reverseYaw ? "true" : "false");
    });
    Telemetry::subscribe("imu/command", [](const char *topic, const char *data, int data_len) // TODO: THis needs to be deprecated for main command handler
                         {
        // check if data is a string
        if (data_len <= 0) {
            ESP_LOGW(TAG, "Received empty command for IMU");
            return;
        }
        
        // Create null-terminated string from MQTT data
        std::string command(data, data_len);
        
        if (command == "calibrate") {
            IMUHelper::calibrate();
            ESP_LOGI(TAG, "IMU calibration started");
        } else if (command == "tare") {
            IMUHelper::tare();
            ESP_LOGI(TAG, "IMU tared");
        } else if (command == "reset_yaw") {
            IMUHelper::resetYaw();
            ESP_LOGI(TAG, "IMU yaw reset");
        } else {
            ESP_LOGW(TAG, "Unknown IMU command: '%s' (length: %d)", command.c_str(), data_len);
        } });

    // Register telemetry callbacks for IMU data
    Telemetry::registerPeriodicCallback([]()
                                        {
                                            // Publish IMU data

                                            Telemetry::publishData("imu/euler", IMUHelper::euler.toString());
                                        },
                                        PublishFrequency::HZ_10);
}
void IMUHelper::tare()
{
    imu.rpt.rv.tare_clear();
    imu.rpt.rv_game.tare(true, true, true);
    imu.rpt.rv.tare_persist();
    // imu.rpt.rv_geomagnetic.tare(true, true, true);
}
void IMUHelper::start()
{
    imu.rpt.rv_game.enable(1000UL);
    imu.rpt.rv_ARVR_stabilized.enable(1000UL);

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
            case SH2_ARVR_STABILIZED_RV:
                euler = imu.rpt.rv_ARVR_stabilized.get_euler(false);
                euler.z -= IMUHelper::yawOffset; // Adjust yaw with offset
                if (IMUHelper::reverseYaw)
                {
                    euler.z = -euler.z;
                }
                // ESP_LOGW(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg] accuracy: %.2f", euler.x*RAD_2_DEG, euler.y*RAD_2_DEG,
                // euler.z*RAD_2_DEG, euler.rad_accuracy*RAD_2_DEG);
                break;
            default:

                break;
            }
        });
}

double IMUHelper::getYaw(bool inDegrees)
{
    return inDegrees ? IMUHelper::euler.z * RAD_2_DEG : IMUHelper::euler.z;
}