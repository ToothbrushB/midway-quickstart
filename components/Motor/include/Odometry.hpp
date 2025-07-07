#include <cstdint>
#include <string>

#pragma once

class Twist2d {
public:
    Twist2d(double dx, double dy, double dtheta)
        : dx(dx), dy(dy), dtheta(dtheta) {}

    // Getters for the twist components
    double getDx() const { return dx; }
    double getDy() const { return dy; }
    double getDtheta() const { return dtheta; }

    // Method to convert to a string representation
    std::string toString() const {
        return "Twist2d(dx: " + std::to_string(dx) +
               ", dy: " + std::to_string(dy) +
               ", dtheta: " + std::to_string(dtheta) + ")";
    }
protected:
    double dx;       // Change in X coordinate in cm
    double dy;       // Change in Y coordinate in cm
    double dtheta;   // Change in heading in radians
};

class Pose2d {
public:
    Pose2d(double x = 0.0, double y = 0.0, double heading = 0.0)
        : x(x), y(y), heading(heading) {}

    // Getters for the pose components
    double getX() const { return x; }
    double getY() const { return y; }
    double getHeading() const { return heading; }

    // Method to convert to a string representation
    std::string toString() const {
        return "Pose2d(x: " + std::to_string(x) +
               ", y: " + std::to_string(y) +
               ", heading: " + std::to_string(heading) + ")";
    }

    Pose2d exp(const Twist2d& twist) const {
        // Apply the twist to the current pose

        double sinTheta = sin(twist.getDtheta());
        double cosTheta = cos(twist.getDtheta());
        double s;
        double c;
        if (abs(twist.getDtheta()) < 1e-9) {
            s = 1.0 - 1.0/6.0 * twist.getDtheta() * twist.getDtheta();
            c = 0.5 * twist.getDtheta();
        } else {
            s = sinTheta/twist.getDtheta();
            c = (1.0-cosTheta)/twist.getDtheta();
        }
        double dx = (twist.getDx() * s - twist.getDy() * c);
        double dy = (twist.getDx() * c + twist.getDy() * s);
        double newX = dx*cos(heading) - dy*sin(heading) + x;
        double newY = dx*sin(heading) + dy*cos(heading) + y;
        double newHeading = heading + twist.getDtheta();
        return Pose2d(newX, newY, newHeading);
    }
protected:
    double x;        // X coordinate in cm
    double y;        // Y coordinate in cm
    double heading;  // Heading in radians
};

class Odometry {
public:
    Odometry();
    void setup(double wheelRadius, double ticksPerRevolution);
    Pose2d update(double vLeft, double vRight, double headingRad, uint64_t tt);
    void reset(Pose2d pose = {0.0, 0.0, 0.0});
    Pose2d getCurrentPose() const {
        return currentPose;
    }
protected:
    Pose2d currentPose;
    double lastUpdateTime;
    double previousHeading;
};