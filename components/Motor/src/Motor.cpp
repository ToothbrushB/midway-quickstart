#include "Motor.hpp" // Include the header file for the Motor class
#include "driver\ledc.h"
#include "driver\gpio.h"
#include "esp_log.h"
#include "ESP32Encoder.h"
#include <cmath>
#include <esp_timer.h>
#include <Telemetry.hpp>
#include <string>
#include "SettingsHelper.hpp"
#define MOTOR_PWM_FREQ 1000 // 1 kHz PWM frequency
#define MOTOR_PWM_RES LEDC_TIMER_10_BIT
#define MOTOR_PWM_MODE LEDC_HIGH_SPEED_MODE

#define Ts 1000 // usec
#define RATIO 25317 // 44 counts per rev, 600 reduction; measured 25317 ish
double WHEEL_RADIUS = 0.033;
#define PI 3.14159265358979

//define signum function
#define signum(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))

// setup tag for esp logging
static const char* TAG = "Motor";
Motor::Motor(char* name): name(name)
{
}

// make a method that setup the motor pwm based on the pins that are passed into the method
void Motor::setup(gpio_num_t in1, gpio_num_t in2, gpio_num_t pwmPin, ledc_timer_t timer, ledc_channel_t channel, ESP32Encoder& encoder)
{
    this->channel = channel;
    motorIn1 = in1;
    motorIn2 = in2;
    ESP_ERROR_CHECK(gpio_reset_pin(pwmPin));
    ESP_ERROR_CHECK(gpio_reset_pin(in1));
    ESP_ERROR_CHECK(gpio_reset_pin(in2));
    // Set the GPIOs for IN1 and IN2
    ESP_ERROR_CHECK(gpio_set_direction(in1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(in2, GPIO_MODE_OUTPUT));

    // Set pwm variable to the pin number from the enum
    int pwm = pwmPin;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = MOTOR_PWM_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = timer,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false};
    ledc_timer_config(&ledc_timer);

    // Set the PWM GPIO
    ledc_channel_config_t ledc_channel = {
        .gpio_num = pwm,
        .speed_mode = MOTOR_PWM_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
        .flags = {
            .output_invert = 0}};
    ledc_channel_config(&ledc_channel);

    this->encoder = &encoder;
    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            Motor* motor = static_cast<Motor*>(arg);
            motor->checkEncoder(); // Call the velocity calculation method
            motor->calculatePID(); // Call the PID calculation method
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "MotorVelocityTimer",
        .skip_unhandled_events = false
    };
    esp_timer_create(&timer_args, &velocityTimer);
    esp_timer_start_periodic(velocityTimer, Ts);


    char key[64];
    snprintf(key, sizeof(key), "M/%s/encrev", name);
    SettingsHelper::addBoolSetting(key, false);
    snprintf(key, sizeof(key), "M/%s/kP", name);
    SettingsHelper::addDoubleSetting(key, .9549297);
    snprintf(key, sizeof(key), "M/%s/kI", name);
    SettingsHelper::addDoubleSetting(key, .47746485);
    snprintf(key, sizeof(key), "M/%s/kD", name);
    SettingsHelper::addDoubleSetting(key, 0.0);
    snprintf(key, sizeof(key), "M/%s/kS", name);
    SettingsHelper::addDoubleSetting(key, 0.24);
    snprintf(key, sizeof(key), "M/%s/kV", name);
    SettingsHelper::addDoubleSetting(key, 1.0);

    // Set PID constants using the same key buffer
    double kP, kI, kD, kS, kV;
    snprintf(key, sizeof(key), "M/%s/kP", name);
    kP = SettingsHelper::getDoubleSetting(key);
    snprintf(key, sizeof(key), "M/%s/kI", name);
    kI = SettingsHelper::getDoubleSetting(key);
    snprintf(key, sizeof(key), "M/%s/kD", name);
    kD = SettingsHelper::getDoubleSetting(key);
    snprintf(key, sizeof(key), "M/%s/kS", name);
    kS = SettingsHelper::getDoubleSetting(key);
    snprintf(key, sizeof(key), "M/%s/kV", name);
    kV = SettingsHelper::getDoubleSetting(key);
    setPIDConstants(kP, kI, kD, kS, kV);
    snprintf(key, sizeof(key), "M/%s/encrev", name);
    encoderReverse = SettingsHelper::getBoolSetting(key);

    SettingsHelper::addDoubleSetting("wheelRadius", WHEEL_RADIUS);
    WHEEL_RADIUS = SettingsHelper::getDoubleSetting("wheelRadius");

    snprintf(key, sizeof(key), "M/%s/kP", name);
    SettingsHelper::registerDoubleCallback(key, [this](const std::pair<const char*, double>& setting) {
        pidConfig.kP = setting.second;
        ESP_LOGI(TAG, "Set kP to %f", pidConfig.kP);
    });
    snprintf(key, sizeof(key), "M/%s/kI", name);
    SettingsHelper::registerDoubleCallback(key, [this](const std::pair<const char*, double>& setting) {
        pidConfig.kI = setting.second;
        ESP_LOGI(TAG, "Set kI to %f", pidConfig.kI);
    });
    snprintf(key, sizeof(key), "M/%s/kD", name);
    SettingsHelper::registerDoubleCallback(key, [this](const std::pair<const char*, double>& setting) {
        pidConfig.kD = setting.second;
        ESP_LOGI(TAG, "Set kD to %f", pidConfig.kD);
    });
    snprintf(key, sizeof(key), "M/%s/kS", name);
    SettingsHelper::registerDoubleCallback(key, [this](const std::pair<const char*, double>& setting) {
        pidConfig.kS = setting.second;
        ESP_LOGI(TAG, "Set kS to %f", pidConfig.kS);
    });
    snprintf(key, sizeof(key), "M/%s/kV", name);
    SettingsHelper::registerDoubleCallback(key, [this](const std::pair<const char*, double>& setting) {
        pidConfig.kV = setting.second;
        ESP_LOGI(TAG, "Set kV to %f", pidConfig.kV);
    });
    SettingsHelper::registerDoubleCallback("wheelRadius", [this](const std::pair<const char*, double>& setting) {
        WHEEL_RADIUS = setting.second;
        ESP_LOGI(TAG, "Set wheel radius to %f", WHEEL_RADIUS);
    });
    snprintf(key, sizeof(key), "M/%s/encrev", name);
    SettingsHelper::registerBoolCallback(key, [this](const std::pair<const char*, bool>& setting) {
        encoderReverse = setting.second;
        ESP_LOGI(TAG, "Set encoder reverse to %s", encoderReverse ? "true" : "false");
    });
    ESP_LOGI(TAG, "Callback registered for motor %s key %s", name, key);

    Telemetry::registerPeriodicCallback([this]() {
        publishTelemetry();
    }, PublishFrequency::HZ_10);
    ESP_LOGI(TAG, "Motor %s initialized successfully", name);
}

void Motor::publishTelemetry() {
    int distanceTicks = getDistanceTicks();

    // publish speed, distanceTicks, output, setpoint, and hasPower as json to telemetry
    char json[256];
    snprintf(json, sizeof(json), "{\"speed\":%f,\"ticks\":%d,\"output\":%f,\"setpoint\":%f,\"hasPower\":%s}",
             motor_speed, distanceTicks, output, pidSetpoint, _hasPower ? "true" : "false");
    char topic[64];
    snprintf(topic, sizeof(topic), "M/%s", name);
    Telemetry::publishData(topic, json, 0);


}

void Motor::testDirection() {
    ESP_LOGI(TAG, "Testing motor direction for %s. Motor will run at half speed for 0.5 seconds.", name);
    set(0.5);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (motor_speed < -0.01) {
        char key[64];
        snprintf(key, sizeof(key), "M/%s/encrev", name);
        encoderReverse = SettingsHelper::getBoolSetting(key);
        SettingsHelper::setBoolSetting(key, !encoderReverse);
        encoderReverse = !encoderReverse;
        ESP_LOGI(TAG, "Reversing encoder direction. Motor speed: %f", motor_speed);
    } else if (motor_speed > 0.01) {
        ESP_LOGI(TAG, "Direction is correct. Motor speed: %f", motor_speed);
    } else {
        ESP_LOGW(TAG, "Motor did not move. Check wiring. Motor speed: %f", motor_speed);
    }
    set(0);
}
PIDConfig Motor::calibrate() {
    //calculate kS
    set(0);
    double i = 0;
    while (motor_speed < 0.001) {
        i += 0.01;
        set(i);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    double kS = i;

    // calculate kV
    double output[20] = {0};
    double speed[20] = {0};
    for (int j = 0; j < 20; ++j) {
        output[j] = kS + j * (1 - kS) / 20.0;
        set(output[j]);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        speed[j] = motor_speed;
    } 

    //perform least-squares regression to determine kV, the slope of the line predicting speed from output
    double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXX = 0.0;
    for (int j = 0; j < 20; ++j) {
        sumX += output[j];
        sumY += speed[j];
        sumXY += output[j] * speed[j];
        sumXX += output[j] * output[j];
    }
    double kV = (20 * sumXY - sumX * sumY) / (20 * sumXX - sumX * sumX);

    return PIDConfig{0.0,0.0,0.0,kS,kV};
}


int64_t Motor::getDistanceTicks()
{
        if (!encoder) {
        ESP_LOGE(TAG, "Encoder not initialized!");
        return 0;
    }
    return encoder->getCount() * (encoderReverse ? -1 : 1);
}

void Motor::setPIDConstants(PIDConfig pidConfig)
{
    this->pidConfig = pidConfig;
    // Reset PID state variables
    integral = 0.0;
    previousError = 0.0;
    previousTime = 0.0; // Initialize previous time
}

void Motor::setPIDConstants(double kP, double kI, double kD, double kS, double kV)
{
    setPIDConstants({kP, kI, kD, kS, kV});
}

void Motor::setReferenceRadPerSec(double radPerSec)
{
    doPid = true;
    pidSetpoint = radPerSec; // Set the target speed in rpm
    // ESP_LOGI(TAG, "PID Setpoint: %f", pidSetpoint);
}

void Motor::setReferenceRpm(double rpm)
{
    setReferenceRadPerSec(rpm * 2 * 3.14159265358979 / 60.0); // Convert rpm to rad/sec
}

void Motor::setReferenceMetersPerSec(double metersPerSec)
{
    setReferenceRadPerSec(metersPerSec / WHEEL_RADIUS);
}

void Motor::checkEncoder()
{

    counts = encoder->getCount() * (encoderReverse ? -1 : 1); // Get the current encoder count
    // diffCounts = counts-prevCounts;
    // prevCounts = counts;
    unsigned long tt = esp_timer_get_time();
    sum -= xbuffer.x[ind];   // (k-(N-1)) will be replaced by k+1 for N-buffer
    xbuffer.x[ind] = counts;
    xbuffer.t[ind] = tt;
    sum += xbuffer.x[ind];
    moment += MOTOR_BUFFER_SIZE*xbuffer.x[ind]-sum;
    a_ = ((4*MOTOR_BUFFER_SIZE-2)*(float)sum - 6*moment)/(MOTOR_BUFFER_SIZE*(MOTOR_BUFFER_SIZE+1));
    motor_speed = (-6*(MOTOR_BUFFER_SIZE-1)*(float)sum + 12*moment)/(MOTOR_BUFFER_SIZE*(MOTOR_BUFFER_SIZE+1))/(Ts*MOTOR_BUFFER_SIZE);
    pos = (a_ + motor_speed*Ts*MOTOR_BUFFER_SIZE)/(RATIO/360.0)*2*3.14159265358979;  // rotations
    motor_speed *= 1000000.0/RATIO*2*3.14159265358979;                        // rad psec

    // ESP_LOGI(TAG, "sum: %ld, Time: %lu, ind: %d, ind[0]: %ld, ind[1]: %ld, ind[2]: %ld, ind[3]: %ld, ind[4]: %ld", 
    //          sum, tt, ind, xbuffer.x[0], xbuffer.x[1], xbuffer.x[2], xbuffer.x[3], xbuffer.x[4]);

++ind;
  ind = ind % MOTOR_BUFFER_SIZE;

//   if (ind == 0)
//   ESP_LOGI(TAG, "Encoder Position: %lld, Time: %lu, Motor Speed: %f, Acceleration: %f, Position: %f", 
//            counts, tt, motor_speed, a_, pos);
}

double Motor::getMotorSpeed() {
    return motor_speed; // Return the current motor speed
}

double Motor::getMotorSpeedMetersPerSec() {
    return motor_speed * WHEEL_RADIUS; // Convert rad/sec to meters/sec
}

// Calculate the PID control output
// must be called ever loop iteration to update the motor speed
// This function calculates the PID output based on the current encoder count and the setpoint
double Motor::calculatePID()
{
    // _hasPower = !(output > 0.1 && motor_speed < 0.0001);

    if (!doPid || !encoder) {
        return 0.0; // Return 0 if PID control is not enabled or encoder is not initialized
    }

    int64_t currentTime = esp_timer_get_time(); // Get the current time in microseconds
    previousTime = currentTime;

    error = pidSetpoint - motor_speed; // Calculate the error

    double derivative = 0.0; // Initialize derivative term
    integral += error*Ts/1e6; // Update the integral term
    derivative = (error - previousError) / Ts*1e6; // Calculate the derivative term
    previousError = error; // Update the previous error
    

    // Calculate PID output
    double pidOutput = pidConfig.kP * error + pidConfig.kI * integral + pidConfig.kD * derivative + pidSetpoint*pidConfig.kV + signum(pidSetpoint)*pidConfig.kS;

    // Set the motor speed based on the PID output
    setInternal(pidOutput);

    // ESP_LOGI(TAG, "PID Output: %f, Setpoint: %f, Current Speed: %f, Error: %f, dT: %f, P: %f, I: %f: D: %f", output, pidSetpoint, motor_speed, error, deltaTime, pidConfig.kP * error, pidConfig.kI * integral, pidConfig.kD * derivative);
    return pidOutput;
}

// speed: [-1,1]
void Motor::set(double speed)
{
    doPid = false;
    setInternal(speed);
}

void Motor::setInternal(double speed) {
    if (speed > 0)
    {
        gpio_set_level(motorIn1, 1);
        gpio_set_level(motorIn2, 0);
    }
    else if (speed < 0)
    {
        gpio_set_level(motorIn1, 0);
        gpio_set_level(motorIn2, 1);
    } else {
        brake();
        return;
    }
    if (speed > 1)
        speed = 1;
    else if (speed < -1)
        speed = -1;
    output = speed;
    ledc_set_duty(MOTOR_PWM_MODE, channel, (int)(abs(speed)*1024));
    ledc_update_duty(MOTOR_PWM_MODE, channel);
}

void Motor::brake()
{
    doPid = false;
    gpio_set_level(motorIn1, 1);
    gpio_set_level(motorIn2, 1);
    ledc_set_duty(MOTOR_PWM_MODE, channel, 0);
    ledc_update_duty(MOTOR_PWM_MODE, channel);
}

void Motor::stop()
{
    doPid = false;
    gpio_set_level(motorIn1, 0);
    gpio_set_level(motorIn2, 0);
    ledc_set_duty(MOTOR_PWM_MODE, channel, 0);
    ledc_update_duty(MOTOR_PWM_MODE, channel);
}
