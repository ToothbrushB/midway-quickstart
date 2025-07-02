#include "Motor.hpp" // Include the header file for the Motor class
#include "driver\ledc.h"
#include "driver\gpio.h"

#define MOTOR_PWM_FREQ 100 // 100 Hz PWM frequency
#define MOTOR_PWM_RES LEDC_TIMER_13_BIT
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_CHANNEL LEDC_CHANNEL_0

#define MOTOR_IN1_GPIO GPIO_NUM_32 // GPIO for H-Bridge IN1
#define MOTOR_IN2_GPIO GPIO_NUM_33 // GPIO for H-Bridge IN2
#define MOTOR_PWM_GPIO 16          // GPIO for PWM output

Motor::Motor()
{
    // Configure IN1 and IN2 as outputs
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_IN1_GPIO) | (1ULL << MOTOR_IN2_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Configure LEDC PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = MOTOR_PWM_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = MOTOR_PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = MOTOR_PWM_GPIO,
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
        .flags = {
            .output_invert = 1}};
    ledc_channel_config(&ledc_channel);
}

// dir: true = forward, false = reverse
// speed: 0-8191 (for 13-bit resolution)
void Motor::set(bool dir, uint32_t speed)
{
    if (dir)
    {
        gpio_set_level(MOTOR_IN1_GPIO, 1);
        gpio_set_level(MOTOR_IN2_GPIO, 0);
    }
    else
    {
        gpio_set_level(MOTOR_IN1_GPIO, 0);
        gpio_set_level(MOTOR_IN2_GPIO, 1);
    }
    if (speed > 8191)
        speed = 8191;
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL, speed);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL);
}

void Motor::brake()
{
    gpio_set_level(MOTOR_IN1_GPIO, 1);
    gpio_set_level(MOTOR_IN2_GPIO, 1);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL);
}

void Motor::stop()
{
    gpio_set_level(MOTOR_IN1_GPIO, 0);
    gpio_set_level(MOTOR_IN2_GPIO, 0);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL);
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