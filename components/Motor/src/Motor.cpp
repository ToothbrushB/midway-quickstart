#include "Motor.hpp" // Include the header file for the Motor class
#include "driver\ledc.h"
#include "driver\gpio.h"
#include "esp_log.h"
#include "ESP32Encoder.h"
#include <math.h>
#include <esp_timer.h>
#include <Telemetry.hpp>

#define MOTOR_PWM_FREQ 25000 // 25 kHz PWM frequency
#define MOTOR_PWM_RES LEDC_TIMER_10_BIT
#define MOTOR_PWM_MODE LEDC_HIGH_SPEED_MODE

#define Ts 1000 // usec
#define TELEMETRY_PERIOD 50000 // usec
#define RATIO 26.4

// setup tag for esp logging
static const char* TAG = "Motor";
Motor::Motor(const char* name):
name(name)
{
}

// make a method that setup the motor pwm based on the pins that are passed into the method
void Motor::setup(gpio_num_t in1, gpio_num_t in2, int pwm, ledc_timer_t timer, ledc_channel_t channel)
{
    this->channel = channel;
    motorIn1 = in1;
    motorIn2 = in2;
    ESP_ERROR_CHECK(gpio_reset_pin(in1));
    ESP_ERROR_CHECK(gpio_reset_pin(in2));
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

    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            Motor* motor = static_cast<Motor*>(arg);
            motor->publishTelemetry(); // Call the publishTelemetry method
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "MotorTelemetryTimer",
        .skip_unhandled_events = false
    };
    esp_timer_create(&timer_args, &telemetryTimer);
    esp_timer_start_periodic(telemetryTimer, TELEMETRY_PERIOD);
}

void Motor::publishTelemetry() {
    int64_t distanceTicks = getDistanceTicks();
    double speed = getMotorSpeed();

    // // Publish telemetry data
    // size_t nbytes = snprintf(NULL, 0, "Speed: %f, Ticks: %lld, output: %f", speed, distanceTicks, output) + 1; /* +1 for the '\0' */
    // char *str = (char*)malloc(nbytes);
    // snprintf(str, nbytes, "Speed: %f, Ticks: %lld, output: %f", speed, distanceTicks, output);

    // Telemetry::publishData(name, str);
    // free(str); // Free the allocated memory after publishing
    ESP_LOGI(TAG, "Motor: %s, Speed: %f, Ticks: %lld, Output: %f, RMS Error: %f", name, speed, distanceTicks, output, rmsError);

}

int64_t Motor::getDistanceTicks()
{
        if (!encoder) {
        ESP_LOGE(TAG, "Encoder not initialized!");
        return 0;
    }
    return encoder->getCount();
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

void Motor::setReferenceRpm(double rpm)
{
    pidSetpoint = rpm; // Set the target speed in rpm
    ESP_LOGI(TAG, "PID Setpoint: %f", pidSetpoint);
}

void Motor::checkEncoder()
{

    counts = encoder->getCount(); // Get the current encoder count
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
    pos = (a_ + motor_speed*Ts*MOTOR_BUFFER_SIZE)/(RATIO*1000.0/360.0);  // ticks
    motor_speed *= 60000.0/RATIO;                        // ticks per min (us to min)

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



void Motor::addEncoder(ESP32Encoder& encoder)
{
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
}

// Calculate the PID control output
// must be called ever loop iteration to update the motor speed
// This function calculates the PID output based on the current encoder count and the setpoint
double Motor::calculatePID()
{
    int64_t currentTime = esp_timer_get_time(); // Get the current time in microseconds
    previousTime = currentTime;

    error[i] = pidSetpoint - motor_speed; // Calculate the error

    double derivative = 0.0; // Initialize derivative term
    integral += error[i]*Ts/1e6; // Update the integral term
    derivative = (error[i] - previousError) / Ts*1e6; // Calculate the derivative term
    previousError = error[i]; // Update the previous error
    

    // Calculate PID output
    double pidOutput = pidConfig.kP * error[i] + pidConfig.kI * integral + pidConfig.kD * derivative + pidSetpoint*pidConfig.kF;

    // Set the motor speed based on the PID output
    set(pidOutput);

    // calculator RMS error of the buffer and store in variable
    rmsError = 0.0;
    for (int j = 0; j < ERROR_BUFFER_SIZE; ++j) {
        rmsError += error[j] * error[j];
    }
    rmsError = sqrt(rmsError / ERROR_BUFFER_SIZE);

    ++i;
    i = i % ERROR_BUFFER_SIZE; // Wrap around the error buffer index

    // ESP_LOGI(TAG, "PID Output: %f, Setpoint: %f, Current Speed: %f, Error: %f, dT: %f, P: %f, I: %f: D: %f", output, pidSetpoint, motor_speed, error, deltaTime, pidConfig.kP * error, pidConfig.kI * integral, pidConfig.kD * derivative);
    return pidOutput;
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
    output = speed;
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