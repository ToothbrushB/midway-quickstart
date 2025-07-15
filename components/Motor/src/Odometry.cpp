#include "Odometry.hpp"
#include <cmath>
#include "Telemetry.hpp"

Pose2d Odometry::currentPose(0.0, 0.0, 0.0); // Initialize current pose
double Odometry::lastUpdateTime = 0.0; // Initialize last update time
double Odometry::previousHeading = 0.0; // Initialize previous heading
#define ROBOT_WIDTH 0.13 // in meters


Pose2d Odometry::update(double vLeft, double vRight, double headingRad, uint64_t tt) {
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

    // Log the updated pose

    Telemetry::publishData("odometry/pose", currentPose.toString().c_str());

    return currentPose;
}

void Odometry::seed(Pose2d pose) {
    // Seed the odometry with a specific pose
    currentPose = pose;
    Telemetry::publishData("odometry/pose", currentPose.toString().c_str());
}

Pose2d Odometry::updateSimple(double vLeft, double vRight, double headingRad, uint64_t tt) {
    // Update the odometry information based on the wheel velocities and current time
    double dt = (tt - lastUpdateTime) / 1000000.0; // Convert microseconds to seconds
    lastUpdateTime = tt;
    if (dt > 1) {
        return getCurrentPose(); // If the time difference is too large, return the last known pose
    }
    double linearVelocity = (vLeft + vRight) / 2.0; // Average of left and right wheel velocities
    double angularVelocity = (vRight - vLeft) / ROBOT_WIDTH; // Difference gives the angular velocity
    currentPose = Pose2d(
        currentPose.getX() + linearVelocity * dt * cos(headingRad),
        currentPose.getY() + linearVelocity * dt * sin(headingRad),
        headingRad // Update heading based on imu
    );
    Telemetry::publishData("odometry/pose", currentPose.toString().c_str());

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
    

    Telemetry::publishData("odometry/pose", currentPose.toString().c_str());

    return currentPose;
}