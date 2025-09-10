#include "BNO08x.hpp"
#include <stdio.h>

#pragma once

class IMUHelper {
public:
    static BNO08x imu;
    static void calibrate();
    static void init();
    static void tare();
    static void start();
    static double getYaw(bool inDegrees = false);
    static void resetYaw();

private:
    static bno08x_euler_angle_t euler;
    static double yawOffset;
    static bool reverseYaw;
};