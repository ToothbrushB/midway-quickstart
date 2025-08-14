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

class LED {
    public:
        void init(int pin_r = 17, int pin_g = 2, int pin_b = 15, ledc_timer_t timer = LEDC_TIMER_1);
        void set_color_rgb(Color c);
        void set_color_hsv(int hue, int saturation, int value);
        void set_color_rgb(int r, int g, int b);
        void off();
        Color hsv2rgb(uint16_t hue, uint8_t saturation, uint8_t value);
        HSVColor rgb2hsv(Color c);

    private:
        int LED_GPIO_PIN_R = 17; // Default GPIO pin for red
        int LED_GPIO_PIN_G = 2; // Default GPIO pin for green;
        int LED_GPIO_PIN_B = 15; // Default GPIO pin for blue
        Color current_color = LED::current_color; // Default color
        void configure_led(void);
};
