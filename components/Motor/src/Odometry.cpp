#include "Odometry.hpp"
#include <cmath>
#include "Telemetry.hpp"
#include "IMUHelper.hpp"
#define ROBOT_WIDTH 0.13 // in meters

static const char* TAG = "Odometry";
std::mutex Odometry::poseMutex; // Mutex for thread-safe access to currentPose
Pose2d Odometry::currentPose(0.0, 0.0, 0.0); // Initialize current pose

double Odometry::lastUpdateTime = 0.0; // Initialize last update time
double Odometry::previousHeading = 0.0; // Initialize previous heading
std::function<double(void)> Odometry::vLeft = []() { return 0.0; }; // Default left wheel velocity function
std::function<double(void)> Odometry::vRight = []() { return 0.0; }; // Default right wheel velocity function
std::function<double(void)> Odometry::heading = []() { return 0.0; }; // Default heading function
esp_timer_handle_t Odometry::timerHandle; // Timer handle for periodic updates
bool Odometry::useImu = false; // Flag to determine if IMU should be used
int Odometry::periodMs = 50; // Default period in milliseconds
void Odometry::timer(void* arg) { //TODO replace w/ normal
    // updateSimple(Odometry::vLeft(), Odometry::vRight(), Odometry::heading(), esp_timer_get_time());
    // if (useImu)
    //     updateSimple(Odometry::vLeft(), Odometry::vRight(), Odometry::heading(), esp_timer_get_time());
    // else
        updateSimpleNoImu(Odometry::vLeft(), Odometry::vRight(), esp_timer_get_time());

}

void Odometry::setup(std::function<double(void)> vLeft, std::function<double(void)> vRight, std::function<double(void)> heading, int periodMs) {
    // Store the wheel velocity and heading functions
    Odometry::vLeft = vLeft;
    Odometry::vRight = vRight;
    Odometry::heading = heading;
    Odometry::periodMs = periodMs;
    esp_timer_create_args_t timer_args = {
        .callback = timer,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "OdometryTimer",
        .skip_unhandled_events = false
    };
    esp_timer_create(&timer_args, &timerHandle);

    char buf[96];
    Telemetry::registerPeriodicCallback([&buf]() {
        // Publish the current pose periodically
        Pose2d pose = Odometry::getCurrentPose();
        pose.toString(buf, sizeof(buf));
        Telemetry::publishData("odometry/pose", buf);
    }, PublishFrequency::HZ_10); // Adjust frequency as needed
}

void Odometry::start() {
    esp_timer_start_periodic(timerHandle, periodMs* 1000); // Convert milliseconds to microseconds
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
    IMUHelper::resetYaw(pose.getHeading());
    ESP_LOGI(TAG, "RESETTING YAW current yaw is %f", IMUHelper::getYaw());
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
    double angularVelocity = (vRight - vLeft) / ROBOT_WIDTH; // Difference gives the angular velocity

    //keep heading within [-pi, pi]
    double encoderHeading = currentPose.getHeading() + angularVelocity * dt;

    double normEncoder = fmod(encoderHeading + 2*M_PI, 2*M_PI);
    double normImu = fmod(headingRad + 2*M_PI, 2*M_PI);
    double diff = normEncoder - normImu;
    diff = fmod(diff + 2*M_PI, 2*M_PI);
    if (fabs(diff) > M_PI) diff = -2*M_PI + diff;
    
    
    double fusedHeading = normImu + diff * 0.5;
    fusedHeading = fmod(fusedHeading + 2*M_PI, 2*M_PI);

    // printf("Fused: %f enc: %f imu: %f\n", fusedHeading*180.0/M_PI, currentPose.getHeading()*180.0/M_PI, headingRad*180.0/M_PI);
    currentPose = Pose2d(
        currentPose.getX() + linearVelocity * dt * cos(headingRad),
        currentPose.getY() + linearVelocity * dt * sin(headingRad),
        headingRad
    );
    return currentPose;
}

Pose2d Odometry::updateSimpleNoImu(double vLeft, double vRight, uint64_t tt) {
        // Update the odometry information based on the wheel velocities and current time
    double dt = (tt - lastUpdateTime) / 1000000.0; // Convert microseconds to seconds
    lastUpdateTime = tt;
    if (dt > 1) {
        return getCurrentPose(); // If the time difference is too large, return the last known pose
    }
    double linearVelocity = (vLeft + vRight) / 2.0; // Average of left and right wheel velocities
    double angularVelocity = (vRight - vLeft) / ROBOT_WIDTH; // Difference gives the angular velocity
    //keep heading within [-pi, pi]
    double newHeading = currentPose.getHeading() + angularVelocity * dt;
    if (newHeading < -M_PI) {
        newHeading += 2 * M_PI;
    } else if (newHeading > M_PI) {
        newHeading -= 2 * M_PI;
    }

    currentPose = Pose2d(
        currentPose.getX() + linearVelocity * dt * cos(currentPose.getHeading()),
        currentPose.getY() + linearVelocity * dt * sin(currentPose.getHeading()),
        newHeading
    );
    return currentPose;
}