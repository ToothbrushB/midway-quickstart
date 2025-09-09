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

static const char *TAG = "LED";
std::vector<Color> colors = {Color{255, 0, 0}, Color{0, 255, 0}, Color{0, 0, 255}};
int step;
int delay_ms = 2500; // default to 1s
bool blink_is_on;

void LED::init(int pin_r, int pin_g, int pin_b, ledc_timer_t timer) 
{
    gpio_reset_pin(static_cast<gpio_num_t>(pin_r));
    gpio_reset_pin(static_cast<gpio_num_t>(pin_g));
    gpio_reset_pin(static_cast<gpio_num_t>(pin_b));
    LED_GPIO_PIN_R = pin_r;
    LED_GPIO_PIN_G = pin_g;
    LED_GPIO_PIN_B = pin_b;

    SettingsHelper::addStringSetting("led_color", "255,0,0"); // Default color red


    // use LedC to control the RGB LED

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = timer,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[3];
    ledc_channel[0] = {
        .gpio_num = LED_GPIO_PIN_R,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
        .flags = {.output_invert = 0}
    };
    ledc_channel[1] = {
        .gpio_num = LED_GPIO_PIN_G,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
        .flags = {.output_invert = 0}
    };
    ledc_channel[2] = {
        .gpio_num = LED_GPIO_PIN_B,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
        .flags = {.output_invert = 0}
    };
    for (int i = 0; i < 3; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }

    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            LED* led = static_cast<LED*>(arg);
            led->blink();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "LedTimer",
        .skip_unhandled_events = false
    };
    // esp_timer_create(&timer_args, &timerHandle);
    // esp_timer_start_periodic(timerHandle, delay_ms * 1000); // Convert milliseconds to microseconds
    // blink_is_on = true;
}

void LED::change_blink_delay(int new_delay) {
    esp_timer_stop(timerHandle); // timer stopped
    delay_ms = new_delay;
    esp_timer_start_periodic(timerHandle, delay_ms * 1000); // Convert milliseconds to microseconds
    blink_is_on = true;
}

void LED::change_blink_pattern(std::vector<Color> new_pattern) {
    colors = new_pattern;
}

void LED::set_blink_on(bool blink_on) {
    if (blink_on && !blink_is_on) {
        esp_timer_stop(timerHandle); // restart timer if true
        esp_timer_start_periodic(timerHandle, delay_ms * 1000); // Convert milliseconds to microseconds
        blink_is_on = true;
    } else if (!blink_on && blink_is_on) {
        esp_timer_stop(timerHandle); // turn off if false
        blink_is_on = false;
    }
}

void LED::set_step(int new_step) {
    step = new_step;
}

void LED::configure_blink(int delay, std::vector<Color> pattern, bool blink_on, int new_step) {
    if (delay != delay_ms) {
        change_blink_delay(delay);
    }
    // check if pattern is null
    if (!pattern.empty() && pattern != colors) {
        change_blink_pattern(pattern);
    }
    set_blink_on(blink_on);
    set_step(new_step);
}

void LED::add_gradient(Color c1, int step1, Color c2, int step2, bool append) {
    for (int i = step1; i <= step2; i++) {
        double ratio = double(i - step1) / (step2 - step1);
        Color c = {
            static_cast<uint8_t>(c1.r + (c2.r - c1.r) * ratio),
            static_cast<uint8_t>(c1.g + (c2.g - c1.g) * ratio),
            static_cast<uint8_t>(c1.b + (c2.b - c1.b) * ratio)
        };
        if (!append && i < colors.size()) {
            colors[i] = c;
        }
        else {
            colors.push_back(c);
        }
    }
}

void LED::blink(void) {
    if (colors.empty()) {
        ESP_LOGW(TAG, "No colors in pattern, skipping blink");
        return;
    }

    set_color_rgb(colors[step]);
    step++;
    // ESP_LOGI(TAG, "LED Color: R=%d, G=%d, B=%d, Step=%d", current_color.r, current_color.g, current_color.b, step);
    if (current_color.r == 255 && current_color.g == 0 && current_color.b == 0) {
        ESP_LOGI(TAG, "LED Color: RED");
    } else if (current_color.r == 0 && current_color.g == 255 && current_color.b == 0) {
        ESP_LOGI(TAG, "LED Color: GREEN");
    } else if (current_color.r == 0 && current_color.g == 0 && current_color.b == 255) {
        ESP_LOGI(TAG, "LED Color: BLUE");
    } else {
        ESP_LOGI(TAG, "LED Color: R=%d, G=%d, B=%d", current_color.r, current_color.g, current_color.b);
    }
    if (step >= colors.size()) {
        step = 0;
        ESP_LOGI(TAG, "LED Pattern Restarted");
    }
    if (delay_ms >= 500 || step % (500 / delay_ms) == 0) { // publish color data every ~500ms or every step (if delay_ms > 500ms)
        char color_str[50];
        snprintf(color_str, sizeof(color_str), "\"r\": %d, \"g\": %d, \"b\": %d, \"step\": %d", current_color.r, current_color.g, current_color.b, step);
        Telemetry::publishData("led", color_str, 0);
    }
}

void LED::set_color_rgb(Color c) {
    current_color = c;

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, (int)(c.r*0.6666667)); // red led only 2.2V
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, (int)(c.g));
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, (int)(c.b));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4);
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

