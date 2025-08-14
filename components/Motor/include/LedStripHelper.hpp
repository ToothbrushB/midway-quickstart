#include "led_strip.h"
#pragma once

struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct HSVColor {
    uint16_t hue;        // 0-360
    uint8_t saturation;  // 0-255
    uint8_t value;       // 0-255
};

class LedStripHelper {
    public:
        static void init(void);
        static void set_color_rgb(Color c);
        static void set_color_hsv(int hue, int saturation, int value);
        static void set_color_rgb(int r, int g, int b);
        static void off();
        static Color hsv2rgb(uint16_t hue, uint8_t saturation, uint8_t value);
        static HSVColor rgb2hsv(Color c);

    private:
        static int LED_STRIP_LED_COUNT;
        static int LED_STRIP_GPIO_PIN;
        static led_strip_handle_t led_strip;
        static led_strip_handle_t configure_led(void);
        static Color current_color;
};
