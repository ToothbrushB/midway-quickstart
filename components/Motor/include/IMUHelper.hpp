#include "BNO08x.hpp"
#include <stdio.h>

#pragma once

class IMUHelper {
public:
    static BNO08x imu;
    static void calibrate();
    static void init();
    static void tare();
    static double getYaw();
    static void start();
private:
    static bno08x_euler_angle_t euler;
};