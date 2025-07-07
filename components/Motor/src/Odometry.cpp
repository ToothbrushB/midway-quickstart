#include "Odometry.hpp"
#include <cmath>

Odometry::Odometry(){}

Pose2d Odometry::update(double vLeft, double vRight, double headingRad, uint64_t tt) {
    // Update the odometry information based on the wheel velocities and heading
    double dt = (tt - lastUpdateTime) / 1000000.0; // Convert microseconds to seconds
    lastUpdateTime = tt;
    double linearVelocity = (vLeft + vRight) / 2.0; // Average of left and right wheel velocities
    double angularVelocity = (vRight - vLeft) / 2.0; // Difference gives the angular velocity
    Twist2d twist = Twist2d(linearVelocity * dt, 0.0, headingRad - previousHeading);
    currentPose = currentPose.exp(twist);
    previousHeading = headingRad;
    return getCurrentPose();
}
