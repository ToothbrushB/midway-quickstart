#pragma once
#include "driver/ledc.h"
#include "esp_mac.h"
#include <vector>
#include "esp_timer.h"

struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    bool operator==(const Color& other) const {
        return r == other.r && g == other.g && b == other.b;
    }
};

struct HSVColor {
    uint16_t hue;        // 0-360
    uint8_t saturation;  // 0-255
    uint8_t value;       // 0-255
};

class LED {
    public:
        void init(int pin_r = 17, int pin_g = 2, int pin_b = 15, ledc_timer_t timer = LEDC_TIMER_1);
        void blink(void);
        void set_color_rgb(Color c);
        void set_color_hsv(int hue, int saturation, int value);
        void set_color_rgb(int r, int g, int b);
        void off();
        Color hsv2rgb(uint16_t hue, uint8_t saturation, uint8_t value);
        HSVColor rgb2hsv(Color c);

        // blink patterns
        void configure_blink(int delay, std::vector<Color> pattern, bool blink_on, int new_step);

        void change_blink_delay(int new_delay);
        void change_blink_pattern(std::vector<Color> new_pattern);
        void set_step(int new_step);
        void set_blink_on(bool blink_on);
        void add_gradient(Color c1, int step1, Color c2, int step2, bool append);

    private:
        int LED_GPIO_PIN_R = 17; // Default GPIO pin for red
        int LED_GPIO_PIN_G = 2; // Default GPIO pin for green;
        int LED_GPIO_PIN_B = 15; // Default GPIO pin for blue
        Color current_color = LED::current_color; // Default color
        void configure_led(void);

    protected:
        esp_timer_handle_t timerHandle; // Timer handle for periodic updates
};
