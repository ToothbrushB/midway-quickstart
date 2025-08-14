#include "Odometry.hpp"
#include <cmath>
#include "Telemetry.hpp"
#define ROBOT_WIDTH 0.13 // in meters

std::mutex Odometry::poseMutex; // Mutex for thread-safe access to currentPose
Pose2d Odometry::currentPose(0.0, 0.0, 0.0); // Initialize current pose
Pose2d Odometry::currentPose2(0.0, 0.0, 0.0); // Initialize current pose

double Odometry::lastUpdateTime = 0.0; // Initialize last update time
double Odometry::lastUpdateTime2 = 0.0; // For the second pose
double Odometry::previousHeading = 0.0; // Initialize previous heading
std::function<double(void)> Odometry::vLeft = []() { return 0.0; }; // Default left wheel velocity function
std::function<double(void)> Odometry::vRight = []() { return 0.0; }; // Default right wheel velocity function
std::function<double(void)> Odometry::heading = []() { return 0.0; }; // Default heading function
esp_timer_handle_t Odometry::timerHandle; // Timer handle for periodic updates

void Odometry::timer(void* arg) {
    updateSimple(Odometry::vLeft(), Odometry::vRight(), Odometry::heading(), esp_timer_get_time());
    updateSimpleNoImu(Odometry::vLeft(), Odometry::vRight(), esp_timer_get_time());
}

void Odometry::setup(std::function<double(void)> vLeft, std::function<double(void)> vRight, std::function<double(void)> heading, int periodMs) {
    // Store the wheel velocity and heading functions
    Odometry::vLeft = vLeft;
    Odometry::vRight = vRight;
    Odometry::heading = heading;
    
    esp_timer_create_args_t timer_args = {
        .callback = timer,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "OdometryTimer",
        .skip_unhandled_events = false
    };
    esp_timer_create(&timer_args, &timerHandle);
    esp_timer_start_periodic(timerHandle, periodMs* 1000); // Convert milliseconds to microseconds

    Telemetry::registerPeriodicCallback([]() {
        // Publish the current pose periodically
        Pose2d pose = Odometry::getCurrentPose();
        Telemetry::publishData("odometry/pose", pose.toString());
        
        Pose2d pose2 = Odometry::getCurrentPose2();
        Telemetry::publishData("odometry/pose2", pose2.toString());
    }, PublishFrequency::HZ_100); // Adjust frequency as needed
}

Pose2d Odometry::update(double vLeft, double vRight, double headingRad, uint64_t tt) {
    std::lock_guard<std::mutex> lock(poseMutex); // Lock for thread safety

    // Update the odometry information based on the wheel velocities and heading
    double dt = (tt - lastUpdateTime) / 1000000.0; // Convert microseconds to seconds
    lastUpdateTime = tt;
    if (dt > 1) {
        return getCurrentPose(); // If the time difference is too large, return the last known pose
    }
    double linearVelocity = (vLeft + vRight) / 2.0; // Average of left and right wheel velocities

    Twist2d twist = Twist2d(linearVelocity * dt, 0.0, headingRad - previousHeading);
    currentPose = currentPose.exp(twist);
    previousHeading = headingRad;

    return currentPose;
}

void Odometry::seed(Pose2d pose) {
    // Seed the odometry with a specific pose
    std::lock_guard<std::mutex> lock(poseMutex); // Lock for thread safety
    currentPose = pose;
}

Pose2d Odometry::updateSimple(double vLeft, double vRight, double headingRad, uint64_t tt) {
    std::lock_guard<std::mutex> lock(poseMutex); // Lock for thread safety
    // Update the odometry information based on the wheel velocities and current time
    double dt = (tt - lastUpdateTime) / 1000000.0; // Convert microseconds to seconds
    lastUpdateTime = tt;
    if (dt > 1) {
        return getCurrentPose(); // If the time difference is too large, return the last known pose
    }
    double linearVelocity = (vLeft + vRight) / 2.0; // Average of left and right wheel velocities
    currentPose = Pose2d(
        currentPose.getX() + linearVelocity * dt * cos(headingRad),
        currentPose.getY() + linearVelocity * dt * sin(headingRad),
        headingRad // Update heading based on imu
    );
    return currentPose;
}

Pose2d Odometry::updateSimpleNoImu(double vLeft, double vRight, uint64_t tt) {
        // Update the odometry information based on the wheel velocities and current time
    double dt = (tt - lastUpdateTime2) / 1000000.0; // Convert microseconds to seconds
    lastUpdateTime2 = tt;
    if (dt > 1) {
        return getCurrentPose2(); // If the time difference is too large, return the last known pose
    }
    double linearVelocity = (vLeft + vRight) / 2.0; // Average of left and right wheel velocities
    double angularVelocity = (vRight - vLeft) / ROBOT_WIDTH; // Difference gives the angular velocity
    //keep heading within [-pi, pi]
    double newHeading = currentPose2.getHeading() + angularVelocity * dt;
    if (newHeading < -M_PI) {
        newHeading += 2 * M_PI;
    } else if (newHeading > M_PI) {
        newHeading -= 2 * M_PI;
    }

    currentPose2 = Pose2d(
        currentPose2.getX() + linearVelocity * dt * cos(currentPose2.getHeading()),
        currentPose2.getY() + linearVelocity * dt * sin(currentPose2.getHeading()),
        newHeading
    );
    return currentPose2;
}