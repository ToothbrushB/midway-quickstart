#include <cstdint>
#include <string>
#include <cmath>
#include <esp_timer.h>
#include <functional>
#include <mutex>
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
    char* toString() const {
        char* buf = new char[128];
        snprintf(buf, 128, "{\"dx\": %.6f, \"dy\": %.6f, \"dtheta\": %.6f}", dx, dy, dtheta);
        return buf;
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
    char* toString(char* buf, size_t len) const {
        snprintf(buf, len, "{\"x\":%.6f,\"y\":%.6f,\"heading\":%.6f}", x, y, heading);
        return buf;
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
    static void setup(std::function<double(void)> vLeft, std::function<double(void)> vRight, std::function<double(void)> heading, int periodMs);
    static void seed(Pose2d pose);
    static Pose2d getCurrentPose() {
        return currentPose;
    }

    static void setUseImu(bool use) {
        useImu = use;
    }
    static void start();

protected:
    static std::mutex poseMutex;
    static Pose2d currentPose;
    static double lastUpdateTime;
    static double previousHeading;
    static Pose2d updateSimple(double vLeft, double vRight, double headingRad, uint64_t tt);
    static Pose2d updateSimpleNoImu(double vLeft, double vRight, uint64_t tt);
    static Pose2d update(double vLeft, double vRight, double headingRad, uint64_t tt);
    static void timer(void* arg);
    static std::function<double(void)> vLeft;
    static std::function<double(void)> vRight;
    static std::function<double(void)> heading;
    static esp_timer_handle_t timerHandle;
    static bool useImu;
    static int periodMs;
};