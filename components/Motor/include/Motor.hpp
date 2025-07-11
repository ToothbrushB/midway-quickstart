#include <cstdint>
#include "driver\ledc.h"
#include "driver\gpio.h"
#include "ESP32Encoder.h"
#include <esp_timer.h>
#pragma once

#define MOTOR_BUFFER_SIZE 50
#define ERROR_BUFFER_SIZE 5

struct PIDConfig {
    double kP;
    double kI;
    double kD;
    double kF;
};

class Motor {
public:
    Motor(const char* name = "Motor");
    void set(double speed);
    void brake();
    void stop();
    void setup(gpio_num_t in1, gpio_num_t in2, int pwm, ledc_timer_t timer, ledc_channel_t channel);
    int64_t getDistanceTicks();
    void setPIDConstants(PIDConfig pidConfig);
    void setReferenceRpm(double rpm);
    double calculatePID();
    void setupMath(double wheelRadius, double ticksPerRevolution);
    void addEncoder(ESP32Encoder& encoder);
    double getMotorSpeed();


private:
    ledc_channel_t channel;
    gpio_num_t motorIn1;
    gpio_num_t motorIn2;
    ESP32Encoder* encoder;

    PIDConfig pidConfig;
    double pidSetpoint = 0.0; // Target speed in rpm
    double integral = 0.0; // Integral term for PID
    double previousError = 0.0; // Previous error for PID
    int64_t previousTime = 0.0; // Previous time for PID calculation
    void checkEncoder();
    long sum = 0;
    struct {
    long x[MOTOR_BUFFER_SIZE] = {0};  // encoder positions 
    unsigned long t[MOTOR_BUFFER_SIZE] = {0};   // time marks in usec.
    } xbuffer;

    double error[ERROR_BUFFER_SIZE] = {0}; // error buffer for PID
    int i=0; // index for error buffer

    double u, a_, pos, motor_speed, moment= 0;
    int64_t counts = 0;
    int ind=0;
    void publishTelemetry();
    const char* name;
    double output;
    double rmsError = 0.0; // Root Mean Square error for PID

    esp_timer_handle_t telemetryTimer;  // Timer for telemetry publishing
    esp_timer_handle_t velocityTimer;   // Timer for velocity calculation

    // int64_t prevCounts = 0;
    // int diffCounts = 0;
};