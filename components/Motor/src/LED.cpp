#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "LED.hpp"
#include "Telemetry.hpp"
#include "SettingsHelper.hpp"
#include "driver/ledc.h"

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

const char *TAG = "LED";

void LED::init(int pin_r, int pin_g, int pin_b, ledc_timer_t timer) 
{
    LED_GPIO_PIN_R = pin_r;
    LED_GPIO_PIN_G = pin_g;
    LED_GPIO_PIN_B = pin_b;

    SettingsHelper::addStringSetting("led_color", "255,0,0"); // Default color red

    Telemetry::registerPeriodicCallback([this]() {
        // Periodic callback code here
        // publish current color to telemetry in rgb using snprintf
        char color_str[32];
        snprintf(color_str, sizeof(color_str), "%d,%d,%d", current_color.r, current_color.g, current_color.b);
        Telemetry::publishData("led/color", color_str, 0);
    }, PublishFrequency::HZ_1); // Adjust frequency as needed
    // use LedC to control the RGB LED

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = timer,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[3];
    ledc_channel[0] = {
        .gpio_num = LED_GPIO_PIN_R,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags = {.output_invert = 0}
    };
    ledc_channel[1] = {
        .gpio_num = LED_GPIO_PIN_G,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags = {.output_invert = 0}
    };
    ledc_channel[2] = {
        .gpio_num = LED_GPIO_PIN_B,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags = {.output_invert = 0}
    };
}


void LED::set_color_rgb(Color c) {
    current_color = c;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (int)(c.r));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, (int)(c.g));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, (int)(c.b));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}

void LED::set_color_hsv(int hue, int saturation, int value) {
    set_color_rgb(hsv2rgb(hue, saturation, value));
}

void LED::set_color_rgb(int r, int g, int b) {
    set_color_rgb(Color{static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)});
}

void LED::off() {
    set_color_rgb(Color{0, 0, 0});
}

Color LED::hsv2rgb(uint16_t hue, uint8_t saturation, uint8_t value) {

    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;

    uint32_t rgb_max = value;
    uint32_t rgb_min = rgb_max * (255 - saturation) / 255.0f;

    uint32_t i = hue / 60;
    uint32_t diff = hue % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        red = rgb_max;
        green = rgb_min + rgb_adj;
        blue = rgb_min;
        break;
    case 1:
        red = rgb_max - rgb_adj;
        green = rgb_max;
        blue = rgb_min;
        break;
    case 2:
        red = rgb_min;
        green = rgb_max;
        blue = rgb_min + rgb_adj;
        break;
    case 3:
        red = rgb_min;
        green = rgb_max - rgb_adj;
        blue = rgb_max;
        break;
    case 4:
        red = rgb_min + rgb_adj;
        green = rgb_min;
        blue = rgb_max;
        break;
    default:
        red = rgb_max;
        green = rgb_min;
        blue = rgb_max - rgb_adj;
        break;
    }
    return {static_cast<uint8_t>(red), static_cast<uint8_t>(green), static_cast<uint8_t>(blue)};
}

HSVColor LED::rgb2hsv(Color c) {
    uint8_t max = std::max(std::max(c.r, c.g), c.b);
    uint8_t min = std::min(std::min(c.r, c.g), c.b);
    uint8_t delta = max - min;

    HSVColor hsv;
    if (delta == 0) {
        hsv.hue = 0;
    } else if (max == c.r) {
        hsv.hue = (60 * ((c.g - c.b) / delta) + 360) % 360;
    } else if (max == c.g) {
        hsv.hue = (60 * ((c.b - c.r) / delta) + 120) % 360;
    } else {
        hsv.hue = (60 * ((c.r - c.g) / delta) + 240) % 360;
    }

    hsv.saturation = (max == 0) ? 0 : (delta * 255 / max);
    hsv.value = max;

    return hsv;
}

