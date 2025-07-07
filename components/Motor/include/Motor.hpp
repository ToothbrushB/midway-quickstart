#include <cstdint>
#include "driver\ledc.h"
#include "driver\gpio.h"
#include "ESP32Encoder.h"
#pragma once

struct PIDConfig {
    double kP;
    double kI;
    double kD;
};

class Motor {
public:
    Motor();
    void set(double speed);
    void brake();
    void stop();
    void setup(gpio_num_t in1, gpio_num_t in2, int pwm, ledc_timer_t timer, ledc_channel_t channel);
    double getDistanceCm();
    void setPIDConstants(PIDConfig pidConfig);
    void setVelocityCmPerSec(double velocityCmPerSec);
    double calculatePID();
    void setupMath(double wheelRadius, double ticksPerRevolution);
    void addEncoder(ESP32Encoder encoder);


private:
    ledc_channel_t channel;
    gpio_num_t motorIn1;
    gpio_num_t motorIn2;
    ESP32Encoder encoder;
    double wheelRadius;
    double ticksPerRevolution;
    PIDConfig pidConfig;
    double pidSetpoint = 0.0; // Target speed in ticks per second
    double integral = 0.0; // Integral term for PID
    double previousError = 0.0; // Previous error for PID
    int64_t previousTime = 0.0; // Previous time for PID calculation
    void checkEncoder();

};