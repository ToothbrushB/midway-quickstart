#include "Motor.hpp" // Include the header file for the Motor class
#include "driver\ledc.h"
#include "driver\gpio.h"
#include "esp_log.h"
#include "ESP32Encoder.h"
#include <math.h>
#include <esp_timer.h>

#define MOTOR_PWM_FREQ 1000 // 1 kHz PWM frequency
#define MOTOR_PWM_RES LEDC_TIMER_10_BIT
#define MOTOR_PWM_MODE LEDC_HIGH_SPEED_MODE

#define N 20
#define Ts 250 // usec

// setup tag for esp logging
static const char* TAG = "Motor";
Motor::Motor()
{
}

// make a method that setup the motor pwm based on the pins that are passed into the method
void Motor::setup(gpio_num_t in1, gpio_num_t in2, int pwm, ledc_timer_t timer, ledc_channel_t channel)
{
    this->channel = channel;
    motorIn1 = in1;
    motorIn2 = in2;
    // Set the GPIOs for IN1 and IN2
    ESP_ERROR_CHECK(gpio_set_direction(in1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(in2, GPIO_MODE_OUTPUT));

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
}

// get encoder distance in cm
// This function should calculate the distance based on the encoder ticks and wheel radius
double Motor::getDistanceCm()
{
    double distance = (encoder.getCount() / ticksPerRevolution) * (2 * M_PI * wheelRadius);
    // ESP_LOGI(TAG, "Distance traveled: %f cm", distance);
    return distance;
}

// set pid constants
void Motor::setPIDConstants(PIDConfig pidConfig)
{
    this->pidConfig = pidConfig;
    // Reset PID state variables
    integral = 0.0;
    previousError = 0.0;
    previousTime = 0.0; // Initialize previous time
}

// set velocity for pid controller
void Motor::setVelocityCmPerSec(double velocityCmPerSec)
{
    pidSetpoint = velocityCmPerSec / (2 * M_PI * wheelRadius) * ticksPerRevolution; // Convert cm/s to ticks/s
}


void Motor::setupMath(double wheelRadius, double ticksPerRevolution)
{
    this->wheelRadius = wheelRadius;
    this->ticksPerRevolution = ticksPerRevolution;
}

long sum = 0;
struct {
  long x[N];  // encoder positions 
  unsigned long t[N];   // time marks in msec.
} xbuffer;

float u, a_, pos, motor_speed, moment= 0;
int64_t counts = 0;
int ind=0;

void Motor::checkEncoder()
{
    counts = encoder.getCount(); // Get the current encoder count
    unsigned long tt = esp_timer_get_time();
    sum -= xbuffer.x[ind];   // (k-(N-1)) will be replaced by k+1 for N-buffer
    xbuffer.x[ind] = counts;
    xbuffer.t[ind] = tt;
    sum += xbuffer.x[ind];
    moment += N*xbuffer.x[ind]-sum;
    a_ = ((4*N-2)*(float)sum - 6*moment)/(N*(N+1));
    motor_speed = (-6*(N-1)*(float)sum + 12*moment)/(N*(N+1))/(Ts*N);
    pos = (a_ + motor_speed*Ts*N)/(1000.0/360.0);  // ticks
    motor_speed *= 60000.0;                        // ticks per sec (us to min)

++ind;
  ind = ind % N;

//   if (ind == 0)
//   ESP_LOGI(TAG, "Encoder Position: %lld, Time: %lu, Motor Speed: %f, Acceleration: %f, Position: %f", 
//            counts, tt, motor_speed, a_, pos);
}



void Motor::addEncoder(ESP32Encoder encoder)
{
    this->encoder = encoder;
    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            Motor* motor = static_cast<Motor*>(arg);
            motor->checkEncoder(); // Call the velocity calculation method
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "MotorVelocityTimer",
        .skip_unhandled_events = false
    };
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, 250);
}

// Calculate the PID control output
// must be called ever loop iteration to update the motor speed
// This function calculates the PID output based on the current encoder count and the setpoint
double Motor::calculatePID()
{
    int64_t currentTime = esp_timer_get_time(); // Get the current time in microseconds
    double deltaTime = (currentTime - previousTime) / 1e6;
    previousTime = currentTime;

    double error = pidSetpoint - motor_speed; // Calculate the error

    double derivative = 0.0; // Initialize derivative term
    if (deltaTime < 500e3) { // only do i and d if time difference is less than 500ms (so don't run i and d on first loop)
        integral += error*deltaTime; // Update the integral term
        double derivative = (error - previousError) / deltaTime; // Calculate the derivative term
        previousError = error; // Update the previous error
    }

    // Calculate PID output
    double output = pidConfig.kP * error + pidConfig.kI * integral + pidConfig.kD * derivative;

    // Set the motor speed based on the PID output
    set(output);

    ESP_LOGI(TAG, "PID Output: %f, Setpoint: %f, Current Speed: %f, Error: %f, dT: %f, P: %f, I: %f: D: %f", output, pidSetpoint, motor_speed, error, deltaTime, pidConfig.kP * error, pidConfig.kI * integral, pidConfig.kD * derivative);
    return output;
}

// speed: [-1,1]
void Motor::set(double speed)
{
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
        stop();
        return;
    }
    if (speed > 1)
        speed = 1;
    else if (speed < -1)
        speed = -1;
    ledc_set_duty(MOTOR_PWM_MODE, channel, (int)(abs(speed)*1024));
    ledc_update_duty(MOTOR_PWM_MODE, channel);
    // ESP_LOGI(TAG, "Motor set to speed: %d", (int)(abs(speed)*1024));
}

void Motor::brake()
{
    gpio_set_level(motorIn1, 1);
    gpio_set_level(motorIn2, 1);
    ledc_set_duty(MOTOR_PWM_MODE, channel, 0);
    ledc_update_duty(MOTOR_PWM_MODE, channel);
}

void Motor::stop()
{
    gpio_set_level(motorIn1, 0);
    gpio_set_level(motorIn2, 0);
    ledc_set_duty(MOTOR_PWM_MODE, channel, 0);
    ledc_update_duty(MOTOR_PWM_MODE, channel);
}

// Example usage
/*
extern "C" void app_main() {
    Motor motor;
    motor.set(true, 512); // Forward at 50% speed
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    motor.set(false, 1023); // Reverse at 100% speed
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    motor.brake();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    motor.stop();
}
*/